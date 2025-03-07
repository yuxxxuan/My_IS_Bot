# Author: ZMAI
# Date: 2025-03-06

import math
import multiprocessing as mp
import time
from multiprocessing import shared_memory
from threading import Thread
import cv2 as cv
import mujoco
import mujoco.viewer
import numpy as np
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from constants import POLICY_CONTROL_PERIOD
from ik_solver import IKSolver
from arm_env_base import ArmEnvBase

# 复用原有的共享内存状态类
class ShmState:
    def __init__(self, existing_instance=None):
        arr = np.empty(3 + 4 + 1 + 1)  # 只需要机械臂相关的状态
        if existing_instance is None:
            self.shm = shared_memory.SharedMemory(create=True, size=arr.nbytes)
        else:
            self.shm = shared_memory.SharedMemory(name=existing_instance.shm.name)
        self.data = np.ndarray(arr.shape, buffer=self.shm.buf)
        self.arm_pos = self.data[:3]  # 机械臂末端位置
        self.arm_quat = self.data[3:7]  # 机械臂末端姿态-四元数
        self.gripper_pos = self.data[7:8]  # 夹爪开合状态
        self.initialized = self.data[8:9]
        self.initialized[:] = 0.0

    def close(self):
        self.shm.close()

# 复用原有的共享内存图像类
class ShmImage:
    def __init__(self, camera_name=None, width=None, height=None, existing_instance=None):
        if existing_instance is None:
            self.camera_name = camera_name
            arr = np.empty((height, width, 3), dtype=np.uint8)
            self.shm = shared_memory.SharedMemory(create=True, size=arr.nbytes)
        else:
            self.camera_name = existing_instance.camera_name
            arr = existing_instance.data
            self.shm = shared_memory.SharedMemory(name=existing_instance.shm.name)
        self.data = np.ndarray(arr.shape, dtype=np.uint8, buffer=self.shm.buf)
        self.data.fill(0)

    def close(self):
        self.shm.close()

# 复用原有的渲染器类
class Renderer:
    def __init__(self, model, data, shm_image):
        self.model = model
        self.data = data
        self.image = np.empty_like(shm_image.data)
        
        # 附加到现有的共享内存图像
        self.shm_image = ShmImage(existing_instance=shm_image)
        
        # 设置相机
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA.value, shm_image.camera_name)
        width, height = model.cam_resolution[camera_id]
        self.camera = mujoco.MjvCamera()
        self.camera.fixedcamid = camera_id
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        
        # 设置上下文
        self.rect = mujoco.MjrRect(0, 0, width, height)
        self.gl_context = mujoco.gl_context.GLContext(width, height)
        self.gl_context.make_current()
        self.mjr_context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN.value, self.mjr_context)
        
        # 设置场景
        self.scene_option = mujoco.MjvOption()
        self.scene = mujoco.MjvScene(model, 10000)
    
    def render(self):
        self.gl_context.make_current()
        mujoco.mjv_updateScene(self.model, self.data, self.scene_option, None, self.camera, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
        mujoco.mjr_render(self.rect, self.scene, self.mjr_context)
        mujoco.mjr_readPixels(self.image, None, self.rect, self.mjr_context)
        self.shm_image.data[:] = np.flipud(self.image)
    
    def close(self):
        self.gl_context.free()
        self.gl_context = None
        self.mjr_context.free()
        self.mjr_context = None

# 机械臂控制器
class ArmController:
    def __init__(self, qpos, qvel, ctrl, qpos_gripper, ctrl_gripper, timestep):
        self.qpos = qpos
        self.qvel = qvel
        self.ctrl = ctrl
        self.qpos_gripper = qpos_gripper
        self.ctrl_gripper = ctrl_gripper
        
        # IK求解器
        self.ik_solver = IKSolver(ee_offset=0.12)
        
        # OTG（在线轨迹生成）
        num_dofs = 7
        self.last_command_time = None
        self.otg = Ruckig(num_dofs, timestep)
        self.otg_inp = InputParameter(num_dofs)
        self.otg_out = OutputParameter(num_dofs)
        self.otg_inp.max_velocity = 4 * [math.radians(80)] + 3 * [math.radians(140)]
        self.otg_inp.max_acceleration = 4 * [math.radians(240)] + 3 * [math.radians(450)]
        self.otg_res = None
        
        # 确保启动时有稳定控制
        self.hold_position = True
    
    def reset(self):
        # 初始化机械臂为"收回"配置
        self.qpos[:] = np.array([0.0, -0.34906585, 3.14159265, -2.54818071, 0.0, -0.87266463, 1.57079633])
        self.ctrl[:] = self.qpos
        self.qpos_gripper[:] = 0.0
        self.ctrl_gripper[:] = 0.0
        
        # 初始化OTG
        self.last_command_time = time.time()
        self.otg_inp.current_position = self.qpos
        self.otg_inp.current_velocity = self.qvel
        self.otg_inp.target_position = self.qpos
        self.otg_res = Result.Finished
        
        # 启用位置保持
        self.hold_position = True
    
    def control_callback(self, command):
        if command is not None:
            self.last_command_time = time.time()
            
            if 'arm_pos' in command:
                # 对新目标姿态运行逆运动学
                qpos = self.ik_solver.solve(command['arm_pos'], command['arm_quat'], self.qpos)
                qpos = self.qpos + np.mod((qpos - self.qpos) + np.pi, 2 * np.pi) - np.pi  # 展开关节角度
                
                # 设置目标机械臂qpos
                self.otg_inp.target_position = qpos
                self.otg_res = Result.Working
                self.hold_position = False  # 收到命令时禁用位置保持
            
            if 'gripper_pos' in command:
                # 设置目标夹爪位置
                self.ctrl_gripper[:] = 255.0 * command['gripper_pos']  # fingers_actuator, ctrlrange [0, 255]
        
        # 如果命令流中断，保持当前姿态
        if time.time() - self.last_command_time > 2.5 * POLICY_CONTROL_PERIOD:
            if not self.hold_position:
                self.otg_inp.target_position = self.otg_out.new_position
                self.otg_res = Result.Working
                self.hold_position = True  # 启用位置保持
        
        # 更新OTG
        if self.otg_res == Result.Working:
            self.otg_res = self.otg.update(self.otg_inp, self.otg_out)
            self.otg_out.pass_to_input(self.otg_inp)
            self.ctrl[:] = self.otg_out.new_position

# MuJoCo仿真类
class MujocoSim:
    def __init__(self, mjcf_path, command_queue, shm_state, show_viewer=True):
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)
        self.data = mujoco.MjData(self.model)
        self.command_queue = command_queue
        self.show_viewer = show_viewer
        
        # 启用重力补偿
        self.model.body_gravcomp[:] = 1.0
        
        # 缓存数组切片引用
        arm_dofs = 7
        qpos_arm = self.data.qpos[:arm_dofs]
        qvel_arm = self.data.qvel[:arm_dofs]
        ctrl_arm = self.data.ctrl[:arm_dofs]
        self.qpos_gripper = self.data.qpos[arm_dofs:(arm_dofs + 1)]
        ctrl_gripper = self.data.ctrl[arm_dofs:(arm_dofs + 1)]
        
        # 控制器
        self.arm_controller = ArmController(qpos_arm, qvel_arm, ctrl_arm, self.qpos_gripper, ctrl_gripper, self.model.opt.timestep)
        
        # 共享内存状态
        self.shm_state = ShmState(existing_instance=shm_state)
        
        # 用于计算机械臂位置和四元数的变量
        site_id = self.model.site('pinch_site').id
        self.site_xpos = self.data.site(site_id).xpos
        self.site_xmat = self.data.site(site_id).xmat
        self.site_quat = np.zeros(4, dtype=np.float64) 
        
        self.base_height = self.model.body('base_link').pos[2]
        self.base_rot_axis = np.array([0.0, 0.0, 1.0])
        self.base_quat_inv = np.zeros(4, dtype=np.float64) 
        # 重置环境
        self.reset()
        
        # 设置控制回调
        mujoco.set_mjcb_control(self.control_callback)
    
    def reset(self):
        # 重置仿真
        mujoco.mj_resetData(self.model, self.data)
        
        # 重置控制器
        self.arm_controller.reset()
        mujoco.mj_forward(self.model, self.data)
        
        # 初始化base_quat_inv
        base_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)  # 默认四元数(w,x,y,z)
        mujoco.mju_negQuat(self.base_quat_inv, base_quat)  # 计算四元数的逆
    
    def control_callback(self, *_):
        # 检查新命令
        command = None if self.command_queue.empty() else self.command_queue.get()
        if command == 'reset':
            self.reset()
        
        # 更新arm quat - 只需要调用一次mju_mat2Quat
        site_xmat_flat = np.array(self.site_xmat, dtype=np.float64).flatten()
        mujoco.mju_mat2Quat(self.site_quat, site_xmat_flat)
        
        # 计算局部坐标系中的四元数
        local_quat = np.zeros(4, dtype=np.float64)
        mujoco.mju_mulQuat(local_quat, self.base_quat_inv, self.site_quat)
        
        # 控制回调
        self.arm_controller.control_callback(command)
        
        # 更新共享内存状态 - 不要重复调用mju_mat2Quat
        self.shm_state.arm_pos[:] = self.site_xpos
        self.shm_state.arm_quat[:] = local_quat  # 使用计算好的局部坐标系四元数
        self.shm_state.gripper_pos[:] = self.qpos_gripper / 255.0  # 归一化夹爪位置
        self.shm_state.initialized[:] = 1.0
    
    def launch(self):
        if self.show_viewer:
            mujoco.viewer.launch(self.model, self.data)
        else:
            while True:
                mujoco.mj_step(self.model, self.data)
                time.sleep(self.model.opt.timestep)

# 主环境类
class KinovaMujocoEnv(ArmEnvBase):
    def __init__(self, render_images=True, show_viewer=True, show_images=False):
        self.mjcf_path = 'models/kinova_gen3/scene_2f85.xml'
        self.render_images = render_images
        self.show_viewer = show_viewer
        self.show_images = show_images
        self.command_queue = mp.Queue(1)
        
        self.hold_position = True 
        
        # 共享内存状态
        self.shm_state = ShmState()
        
        # 共享内存图像
        if self.render_images:
            self.shm_images = []
            model = mujoco.MjModel.from_xml_path(self.mjcf_path)
            for camera_id in range(model.ncam):
                camera_name = model.camera(camera_id).name
                width, height = model.cam_resolution[camera_id]
                self.shm_images.append(ShmImage(camera_name, width, height))
        
        # 启动物理循环
        mp.Process(target=self.physics_loop, daemon=True).start()
        
        if self.render_images and self.show_images:
            # 启动可视化循环
            mp.Process(target=self.visualizer_loop, daemon=True).start()
    
    def physics_loop(self):
        # 创建仿真
        sim = MujocoSim(self.mjcf_path, self.command_queue, self.shm_state, show_viewer=self.show_viewer)
        
        # 启动渲染循环
        if self.render_images:
            Thread(target=self.render_loop, args=(sim.model, sim.data), daemon=True).start()
        
        # 启动仿真
        sim.launch()  # 在创建的同一线程中启动，以避免段错误
    
    def render_loop(self, model, data):
        # 设置渲染器
        renderers = [Renderer(model, data, shm_image) for shm_image in self.shm_images]
        
        # 持续渲染相机图像
        while True:
            start_time = time.time()
            for renderer in renderers:
                renderer.render()
            render_time = time.time() - start_time
            if render_time > 0.1:  # 10 fps
                print(f'警告：离屏渲染耗时 {1000 * render_time:.1f} ms，尝试缩小Mujoco查看器窗口以加速离屏渲染')
    
    def visualizer_loop(self):
        shm_images = [ShmImage(existing_instance=shm_image) for shm_image in self.shm_images]
        last_imshow_time = time.time()
        while True:
            while time.time() - last_imshow_time < 0.1:  # 10 fps
                time.sleep(0.01)
            last_imshow_time = time.time()
            for i, shm_image in enumerate(shm_images):
                cv.imshow(shm_image.camera_name, cv.cvtColor(shm_image.data, cv.COLOR_RGB2BGR))
                cv.moveWindow(shm_image.camera_name, 640 * i, -100)
            cv.waitKey(1)
            
    def reset(self):
        self.shm_state.initialized[:] = 0.0
        self.command_queue.put('reset')

        # Wait for state publishing to initialize
        while self.shm_state.initialized == 0.0:
            time.sleep(0.01)

        # Wait for image rendering to initialize (Note: Assumes all zeros is not a valid image)
        if self.render_images:
            while any(np.all(shm_image.data == 0) for shm_image in self.shm_images):
                time.sleep(0.01)

    def get_obs(self):
        arm_quat = self.shm_state.arm_quat[[1, 2, 3, 0]]  # (w, x, y, z) -> (x, y, z, w)
        if arm_quat[3] < 0.0:  # Enforce quaternion uniqueness
            np.negative(arm_quat, out=arm_quat)
        obs = {
            'arm_pos': self.shm_state.arm_pos.copy(),
            'arm_quat': arm_quat,
            'gripper_pos': self.shm_state.gripper_pos.copy(),
        }
        if self.render_images:
            for shm_image in self.shm_images:
                obs[f'{shm_image.camera_name}_image'] = shm_image.data.copy()
        return obs
    
    def step(self, action):
        # 使用基类的处理方法过滤掉底盘相关动作
        arm_action = self.process_arm_action(action)
        # 调用原有的step方法
        self.command_queue.put(arm_action)

    def close(self):
        self.shm_state.close()
        self.shm_state.shm.unlink()
        if self.render_images:
            for shm_image in self.shm_images:
                shm_image.close()
                shm_image.shm.unlink() 
                
if __name__ =='__main__':
    env = KinovaMujocoEnv()
    
    try:
        while True:
            env.reset()
            for _ in range(100):
                action = {
                    'arm_pos': 0.1 * np.random.rand(3) + np.array([0.55, 0.0, 0.4]),
                    'arm_quat': np.random.rand(4),
                    'gripper_pos': np.random.rand(1),
                }
                env.step(action)
                obs = env.get_obs()
                
                print([(k, v.shape) if v.ndim == 3 else (k, v) for (k, v) in obs.items()])
                time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
    finally:
        env.close()
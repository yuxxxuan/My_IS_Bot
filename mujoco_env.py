# Author: Jimmy Wu
# Date: October 2024
#
# Note: This is a basic simulation environment for sanity checking the
# real-world pipeline for teleop and imitation learning. Performance metrics,
# reward signals, and termination signals are not implemented.

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
from scipy.spatial.transform import Rotation as R
from constants import POLICY_CONTROL_PERIOD
from ik_solver import IKSolver

# 共享内存状态类，用于存储机器人的状态信息
class ShmState:
    # only gen3 1
    def __init__(self, existing_instance=None):
        arr = np.empty(3 + 4 + 1 + 1)  # 创建一个数组用于存储状态信息
        if existing_instance is None: # 如果共享内存不存在，则创建一个新的共享内存
            self.shm = shared_memory.SharedMemory(create=True, size=arr.nbytes)  # 创建共享内存
        else: # 如果共享内存存在，则连接到现有的共享内存
            self.shm = shared_memory.SharedMemory(name=existing_instance.shm.name)  # 连接到现有的共享内存
            
        # 将共享内存映射到NumPy数组
        # self.shm.buf 是该共享内存的缓冲区。它提供了对共享内存的直接访问
        self.data = np.ndarray(arr.shape, buffer=self.shm.buf)  
        
        # 将具体的状态信息存储到数组中
        
        self.arm_pos = self.data[:3]  # 机械臂末端位置
        self.arm_quat = self.data[3:7]  # 机械臂末端姿态-四元数
        self.gripper_pos = self.data[7:8]  # 夹爪开合状态
        self.initialized = self.data[8:9]
        self.initialized[:] = 0.0

    def close(self):
        self.shm.close()  # 关闭共享内存

# 共享内存图像类，用于存储相机图像
class ShmImage:
    def __init__(self, camera_name=None, width=None, height=None, existing_instance=None):
        if existing_instance is None:
            self.camera_name = camera_name
            arr = np.empty((height, width, 3), dtype=np.uint8)  # 创建一个数组用于存储图像
            self.shm = shared_memory.SharedMemory(create=True, size=arr.nbytes)  # 创建共享内存
        else:
            self.camera_name = existing_instance.camera_name
            arr = existing_instance.data
            self.shm = shared_memory.SharedMemory(name=existing_instance.shm.name)  # 连接到现有的共享内存
        # 将共享内存映射到NumPy数组
        self.data = np.ndarray(arr.shape, dtype=np.uint8, buffer=self.shm.buf)  
        self.data.fill(0)  # 初始化图像数据为0

    def close(self):
        self.shm.close()  # 关闭共享内存

# 渲染器类，用于渲染MuJoCo场景
class Renderer:
    def __init__(self, model, data, shm_image):
        self.model = model  # MuJoCo模型
        self.data = data  # MuJoCo数据
        self.image = np.empty_like(shm_image.data)  # 创建图像数组

        # Attach to existing shared memory image
        self.shm_image = ShmImage(existing_instance=shm_image)  # 连接到现有的共享内存图像

        # Set up camera
        camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA.value, shm_image.camera_name)  # 获取相机ID
        width, height = model.cam_resolution[camera_id]  # 获取相机分辨率
        self.camera = mujoco.MjvCamera()  # 创建相机对象
        self.camera.fixedcamid = camera_id  # 设置固定相机ID
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED  # 设置相机类型

        # Set up context
        self.rect = mujoco.MjrRect(0, 0, width, height)  # 创建渲染矩形
        self.gl_context = mujoco.gl_context.GLContext(width, height)  # 创建OpenGL上下文
        self.gl_context.make_current()  # 使上下文当前
        self.mjr_context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)  # 创建MuJoCo上下文
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN.value, self.mjr_context)  # 设置缓冲区

        # Set up scene
        self.scene_option = mujoco.MjvOption()  # 创建场景选项
        self.scene = mujoco.MjvScene(model, 10000)  # 创建场景

    def render(self):
        self.gl_context.make_current()  # 使OpenGL上下文当前
        mujoco.mjv_updateScene(self.model, self.data, self.scene_option, None, self.camera, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)  # 更新场景
        mujoco.mjr_render(self.rect, self.scene, self.mjr_context)  # 渲染场景
        mujoco.mjr_readPixels(self.image, None, self.rect, self.mjr_context)  # 读取像素
        self.shm_image.data[:] = np.flipud(self.image)  # 将图像数据写入共享内存

    def close(self):
        self.gl_context.free()  # 释放OpenGL上下文
        self.gl_context = None
        self.mjr_context.free()  # 释放MuJoCo上下文
        self.mjr_context = None

# 基座控制器类 -> 用于仿真内实时的控制
# class BaseController:
#     def __init__(self, qpos, qvel, ctrl, timestep):
#         self.qpos = qpos  # 位置
#         self.qvel = qvel  # 速度
#         self.ctrl = ctrl  # 控制信号

#         # OTG (online trajectory generation)
#         num_dofs = 3  # 自由度数量
#         self.last_command_time = None  # 上一个命令时间
#         self.otg = Ruckig(num_dofs, timestep)  # 创建在线轨迹生成器
#         self.otg_inp = InputParameter(num_dofs)  # 输入参数
#         self.otg_out = OutputParameter(num_dofs)  # 输出参数
#         self.otg_inp.max_velocity = [0.5, 0.5, 3.14]  # 最大速度
#         self.otg_inp.max_acceleration = [0.5, 0.5, 2.36]  # 最大加速度
#         self.otg_res = None  # 轨迹生成结果

#     def reset(self):
#         # Initialize base at origin
#         self.qpos[:] = np.zeros(3)  # 将位置初始化为原点
#         self.ctrl[:] = self.qpos  # 控制信号设置为位置

#         # Initialize OTG
#         self.last_command_time = time.time()  # 记录当前时间
#         self.otg_inp.current_position = self.qpos  # 当前位置信息
#         self.otg_inp.current_velocity = self.qvel  # 当前速度信息
#         self.otg_inp.target_position = self.qpos  # 目标位置
#         self.otg_res = Result.Finished  # 轨迹生成状态

#     def control_callback(self, command):
#         if command is not None:
#             self.last_command_time = time.time()  # 更新命令时间
#             if 'base_pose' in command:
#                 # Set target base qpos
#                 self.otg_inp.target_position = command['base_pose']  # 设置目标位置
#                 self.otg_res = Result.Working  # 更新状态为工作中

#         # Maintain current pose if command stream is disrupted
#         if time.time() - self.last_command_time > 2.5 * POLICY_CONTROL_PERIOD:
#             self.otg_inp.target_position = self.qpos  # 如果命令中断，保持当前姿态
#             self.otg_res = Result.Working  # 更新状态为工作中

#         # Update OTG
#         if self.otg_res == Result.Working:
#             self.otg_res = self.otg.update(self.otg_inp, self.otg_out)  # 更新轨迹生成
#             self.otg_out.pass_to_input(self.otg_inp)  # 将输出传递给输入
#             self.ctrl[:] = self.otg_out.new_position  # 更新控制信号

# 手臂控制器类 -> 用于仿真内实时的控制
class ArmController:
    def __init__(self, qpos, qvel, ctrl, qpos_gripper, ctrl_gripper, timestep):
        self.qpos = qpos  # 位置
        self.qvel = qvel  # 速度
        self.ctrl = ctrl  # 控制信号
        self.qpos_gripper = qpos_gripper  # 夹爪位置
        self.ctrl_gripper = ctrl_gripper  # 夹爪控制信号

        ## IK solver
        # self.ik_solver = IKSolver(ee_offset=0.12)  # 这个0.12 要不要改？？？ mb
        self.ik_solver = IKSolver(ee_offset=0.12) 

        ## OTG (online trajectory generation)
        num_dofs = 7  # 自由度数量
        self.last_command_time = None  # 上一个命令时间
        self.otg = Ruckig(num_dofs, timestep)  # 创建在线轨迹生成器
        self.otg_inp = InputParameter(num_dofs)  # 输入参数
        self.otg_out = OutputParameter(num_dofs)  # 输出参数
        self.otg_inp.max_velocity = 4 * [math.radians(80)] + 3 * [math.radians(140)]  # 最大速度
        # 这尼玛的设置有点高啊，我查一下再改！ mb ！ math.radians(240) ≈ 4.2 rad/s²
        self.otg_inp.max_acceleration = 4 * [math.radians(240)] + 3 * [math.radians(450)]  # 最大加速度
        self.otg_res = None  # 轨迹生成结果

    def reset(self):
        """ 重置和初始化"""
        # Initialize arm in "retract" configuration
        # 初始化手臂位置 -> 这个单位是弧度
        self.qpos[:] = np.array([0.0, -0.34906585, 3.14159265, -2.54818071, 0.0, -0.87266463, 1.57079633])  # 已测试-折叠初始位置
        self.ctrl[:] = self.qpos  # 控制信号设置为位置
        self.ctrl_gripper[:] = 0.0  # 夹爪控制信号初始化为0

        # Initialize OTG
        self.last_command_time = time.time()  # 记录当前时间
        self.otg_inp.current_position = self.qpos  # 当前位置信息
        self.otg_inp.current_velocity = self.qvel  # 当前速度信息
        self.otg_inp.target_position = self.qpos  # 目标位置
        self.otg_res = Result.Finished  # 轨迹生成状态

    def control_callback(self, command):
        # command -> dict -> 包含arm_pos, arm_quat, gripper_pos
        if command is not None:
            self.last_command_time = time.time()  # 更新命令时间

            if 'arm_pos' in command:
                # using the ik solver to calculate the target joint position
                # 使用逆向运动学求解器计算目标关节位置
                # qpos: 计算得到的关节位置
                # command['arm_pos']: 末端执行器的目标位置
                # command['arm_quat']: 末端执行器的目标四元数
                # self.qpos: 当前末端执行器的位置
                qpos = self.ik_solver.solve(command['arm_pos'], command['arm_quat'], self.qpos)
                
                # 处理关节角度，调整为 [-pi, pi] 范围内,这里感觉也可以处理。 mb
                qpos = self.qpos + np.mod((qpos - self.qpos) + np.pi, 2 * np.pi) - np.pi 

                # print(f"[ArmController-control_callback] [IK]\nTarget arm_pos: {command['arm_pos']} arm_quat: {command['arm_quat']} \n Finished Solve [关节角度] {qpos}")
                print(f"---------------------------------\n"
                      f"[ArmController-control_callback] [IK]\n"
                      f"Target arm_pos: {command['arm_pos']}\n"
                      f"Target arm_quat: {command['arm_quat']}\n"
                      f"IK Solved [关节角度]: {qpos}\n"
                      f"----------------------------------")
                
                # Set target arm qpos -> 给到Ruckig(轨迹生成)
                self.otg_inp.target_position = qpos  # 设置目标手臂位置
                self.otg_res = Result.Working  # 更新状态为工作中

            if 'gripper_pos' in command:
                # Set target gripper pos
                # 缩放因子为255.0？ 这里的依据是什么？
                self.ctrl_gripper[:] = 255.0 * command['gripper_pos']  # 设置夹爪位置

        # Maintain current pose if command stream is disrupted
        # 整个条件的意思是：如果从上一个命令到现在的时间超过了 2.5 倍的控制周期，则执行条件内的代码
        if time.time() - self.last_command_time > 2.5 * POLICY_CONTROL_PERIOD:
            '''
            POLICY_CONTROL_PERIOD -> 10Hz -> 0.1s
            2.5 * POLICY_CONTROL_PERIOD -> 0.25s
            '''
            self.otg_inp.target_position = self.otg_out.new_position  # 如果命令中断，保持当前姿态
            self.otg_res = Result.Working  # 更新状态为工作中

        # Update OTG
        if self.otg_res == Result.Working:
            self.otg_res = self.otg.update(self.otg_inp, self.otg_out)  # 更新轨迹生成，针对机械臂的，夹爪单独控制
            self.otg_out.pass_to_input(self.otg_inp)  # 将输出传递给输入
            self.ctrl[:] = self.otg_out.new_position  # 更新控制信号

# MuJoCo仿真类
class MujocoSim:
    def __init__(self, mjcf_path, command_queue, shm_state, show_viewer=True):
        # only gen3 1
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)  # 从XML路径加载模型
        self.data = mujoco.MjData(self.model)  # 创建数据对象
        self.command_queue = command_queue  # 命令队列
        self.show_viewer = show_viewer  # 是否显示查看器

        # Enable gravity compensation for everything except objects
        self.model.body_gravcomp[:] = 1.0  # 此处1.0 表示启用重力补偿
        

        ## This is the cube 
        # body_names = {self.model.body(i).name for i in range(self.model.nbody)}  # 获取所有物体名称
        # for object_name in ['cube']:
        #     if object_name in body_names:
        #         self.model.body_gravcomp[self.model.body(object_name).id] = 0.0  # 禁用特定物体的重力补偿

        # Cache references to array slices
        
        arm_dofs = 7  # 手臂自由度数量
        qpos_arm = self.data.qpos[: arm_dofs]  # 手臂位置
        qvel_arm = self.data.qvel[: arm_dofs]  # 手臂速度
        ctrl_arm = self.data.ctrl[: arm_dofs] # 手臂控制信号
        self.qpos_gripper = self.data.qpos[arm_dofs:(arm_dofs + 1)]  # 夹爪位置
        ctrl_gripper = self.data.ctrl[arm_dofs:(arm_dofs + 1)]  # 夹爪控制信号
        
        # 这里改的不确定！tag:mb
        # self.qpos_cube = self.data.qpos[(arm_dofs + 8):(arm_dofs + 8 + 7)]  # 立方体位置

        # Controllers
        # self.base_controller = BaseController(self.qpos_base, qvel_base, ctrl_base, self.model.opt.timestep)  # 创建基座控制器
        self.arm_controller = ArmController(qpos_arm, qvel_arm, ctrl_arm, self.qpos_gripper, ctrl_gripper, self.model.opt.timestep)  # 创建手臂控制器

        # Shared memory state for observations
        self.shm_state = ShmState(existing_instance=shm_state)  # 创建共享内存状态

        # Variables for calculating arm pos and quat
        # site 是特定的空间位置？ 为什么？
        site_id = self.model.site('pinch_site').id  # 获取夹爪位置ID
        self.site_xpos = self.data.site(site_id).xpos  # 夹爪位置
        self.site_xmat = self.data.site(site_id).xmat  # 夹爪方向
        self.site_quat = np.empty(4)  # 夹爪四元数
        
        # base? tag:mb
        # self.base_height = self.model.body('gen3/base_link').pos[2]  # 基座高度 
        # self.base_rot_axis = np.array([0.0, 0.0, 1.0])  # 基座旋转轴
        # self.base_quat_inv = np.empty(4)  # 基座四元数的逆

        # Reset the environment
        self.reset()  # 重置环境

        # Set control callback -> 设置控制回调
        # 为set_mjcb_control 设定注册控制回调函数 -> mujoco 每步仿真，均会调用这个函数，以处理控制逻辑
        mujoco.set_mjcb_control(self.control_callback) 

    def reset(self):
        # only gen3 1
        # mb?
        # Reset simulation -> 重置所有仿真数据
        mujoco.mj_resetData(self.model, self.data)  

        ### Reset cube
        # 这里可以插入函数，作为随即更新物体？
        # self.qpos_cube[:2] += np.random.uniform(-0.1, 0.1, 2)  # 随机移动立方体
        # theta = np.random.uniform(-math.pi, math.pi)  # 随机旋转角度
        # self.qpos_cube[3:7] = np.array([math.cos(theta / 2), 0, 0, math.sin(theta / 2)])  # 设置立方体的四元数
        # 设定好cube后，更新仿真状态
        mujoco.mj_forward(self.model, self.data)  

        # Reset controllers -> 重置所有控制器，对物体进行初始化，让控制器基于当前物理状态进行重置
        # self.base_controller.reset()  # 重置基座控制器
        self.arm_controller.reset()  # 重置手臂控制器

    def control_callback(self, *_):
        # tag:mb
        # 检查命令队列是否有新命令
        command = None if self.command_queue.empty() else self.command_queue.get()  # 获取命令
        if command == 'reset':
            self.reset()  # 如果命令是重置，重置仿真

        # Control callbacks
        # 调用控制器的回调函数以处理命令
        # self.base_controller.control_callback(command)  # 处理基座控制
        self.arm_controller.control_callback(command)  # 处理手臂控制


        # site_xpos 是什么类型？有什么内容？感觉不需要？ mb
        # site_xpos = self.site_xpos.copy()  # 复制夹爪位置
        # site_xpos[2] -= self.base_height  # 基座高度偏移
        # site_xpos[:2] -= self.qpos_base[:2]  # 基座位置逆，确保夹爪位置是相对基坐标的
        
        # 基座方向逆
        # self.base_quat_inv -> 基座四元数的逆
        # self.base_rot_axis -> 基座旋转轴
        # self.qpos_base[2] -> 基座旋转角度
        # 目的： 逆向四元数用于将夹爪的位置从全局坐标系转换到基座的局部坐标系?
        # mujoco.mju_axisAngle2Quat(self.base_quat_inv, self.base_rot_axis, -self.qpos_base[2])  
        
        ## 更新手臂位置，局部坐标系中
        # 可能不用了？ tag:mb
        # mujoco.mju_rotVecQuat(self.shm_state.arm_pos, site_xpos, self.base_quat_inv)  

        # Update arm pos -> 更新手臂位置
        self.shm_state.arm_pos[:] = self.site_xpos
        # Update arm quat -> 更新arm四元数 这个应该需要？ mb 03-18 10:37
        self.shm_state.arm_quat[:] = self.site_quat
        
        # mju_mat2Quat 和 mju_mulQuat 没有理解？ mb
        mujoco.mju_mat2Quat(self.site_quat, self.site_xmat)  # site 的相关变量是做什么的？ mb
        # mujoco.mju_mulQuat(self.shm_state.arm_quat, self.base_quat_inv, self.site_quat)  # 手臂四元数在局部坐标系中


        ## Update gripper pos
        # 夹爪位置除以0.8，用于将夹爪控制范围限制在0-1之间
        # 更新共享内容的夹爪状态
        self.shm_state.gripper_pos[:] = self.qpos_gripper / 0.8 
        # Notify reset() function that state has been initialized
        self.shm_state.initialized[:] = 1.0  # 通知状态已初始化

    def launch(self):
        if self.show_viewer:
            # 关闭了左侧和右侧的UI
            mujoco.viewer.launch(
                self.model, 
                self.data, 
                show_left_ui=False, 
                show_right_ui=False
                ) 
        
        else:
            # Run headless simulation at real-time speed
            last_step_time = 0
            while True:
                while time.time() - last_step_time < self.model.opt.timestep:
                    time.sleep(0.0001)  # 等待下一个时间步
                last_step_time = time.time()
                # 更新数据
                mujoco.mj_step(self.model, self.data)  # 执行仿真步骤

# MuJoCo环境创建 -> main.py 调用
class MujocoEnv:
    def __init__(self, render_images=True, show_viewer=True, show_images=False):
        
        # only use the kinova gen3 , dont use the base
        self.mjcf_path = 'models/kinova_gen3/gen3_2f85.xml'  # MuJoCo模型路径
        # self.mjcf_path = 'models/kinova_gen3/mai_room1_1.xml'  # MuJoCo模型路径
        self.render_images = render_images  # 是否渲染图像
        self.show_viewer = show_viewer  # 是否显示查看器
        self.show_images = show_images  # 是否显示 camera image
        # 创建多线程命令队列 -> 用于在物理循环中接收命令
        self.command_queue = mp.Queue(1)  

        # Shared memory for state observations
        self.shm_state = ShmState()  # 初始化共享状态

        # Shared memory for image observations
        if self.render_images:
            self.shm_images = []  # 初始化存储图像的共享内存
            model = mujoco.MjModel.from_xml_path(self.mjcf_path)  # 从XML路径加载模型
            for camera_id in range(model.ncam): # 通过id，便利相机视角内容，并保存到shm_images列表中
                camera_name = model.camera(camera_id).name  # 获取环境内所有相机命名
                width, height = model.cam_resolution[camera_id]  # 获取相机分辨率
                self.shm_images.append(ShmImage(camera_name, width, height))  # 创建共享内存图像

        # Start physics loop
        # 启动新进程，控制物理循环 -> 物理循环负责处理物理引擎的更新，包括重力、碰撞、关节力等
        # target=self.physics_loop -> 指定物理循环的目标函数
        # daemon=True -> 设置为守护进程，当主进程结束时，子进程也会自动结束
        mp.Process(target=self.physics_loop, daemon=True).start()  # 启动物理循环

        # 如果 MujocoEnv 输入的self.render_images and self.show_images 都为True
        if self.render_images and self.show_images:
            # Start visualizer loop
            # 启动新进程，控制可视化循环 -> 可视化循环负责处理图像的渲染和显示
            # target=self.visualizer_loop -> 指定可视化循环的目标函数
            # daemon=True -> 设置为守护进程，当主进程结束时，子进程也会自动结束
            mp.Process(target=self.visualizer_loop, daemon=True).start()  # 启动可视化循环
        
    def physics_loop(self):
        # Create sim
        # 启动 MuJoCoSim 仿真
        print('MujocoEnv-physics_loop: 1')
        # 启用多线程了，里面的东西，没办法print
        sim = MujocoSim(self.mjcf_path, self.command_queue, self.shm_state, show_viewer=self.show_viewer)  

        # Start render loop
        # 启动渲染循环
        if self.render_images:
            Thread(target=self.render_loop, args=(sim.model, sim.data), daemon=True).start()  # 启动渲染循环

        # Launch sim
        sim.launch()  # 启动仿真
    def render_loop(self, model, data):
        # Set up renderers
        renderers = [Renderer(model, data, shm_image) for shm_image in self.shm_images]  # 创建渲染器

        # Render camera images continuously
        while True:
            start_time = time.time()  # 记录开始时间
            for renderer in renderers:
                renderer.render()  # 渲染图像
            render_time = time.time() - start_time  # 计算渲染时间
            if render_time > 0.1:  # 10 fps
                print(f'Warning: Offscreen rendering took {1000 * render_time:.1f} ms, try making the Mujoco viewer window smaller to speed up offscreen rendering')

    def visualizer_loop(self):
        shm_images = [ShmImage(existing_instance=shm_image) for shm_image in self.shm_images]  # 创建共享内存图像
        last_imshow_time = time.time()  # 记录上次显示时间
        while True:
            while time.time() - last_imshow_time < 0.1:  # 10 fps
                time.sleep(0.01)  # 等待
            last_imshow_time = time.time()  # 更新显示时间
            for i, shm_image in enumerate(shm_images):
                cv.imshow(shm_image.camera_name, cv.cvtColor(shm_image.data, cv.COLOR_RGB2BGR))  # 显示图像
                cv.moveWindow(shm_image.camera_name, 640 * i, -100)  # 移动窗口位置
            cv.waitKey(1)  # 等待键盘事件

    def reset(self):
        self.shm_state.initialized[:] = 0.0  # 初始化状态
        self.command_queue.put('reset')  # 发送重置命令

        # Wait for state publishing to initialize
        while self.shm_state.initialized == 0.0:
            time.sleep(0.01)  # 等待状态初始化

        # Wait for image rendering to initialize (Note: Assumes all zeros is not a valid image)
        if self.render_images:
            while any(np.all(shm_image.data == 0) for shm_image in self.shm_images):
                time.sleep(0.01)  # 等待图像渲染初始化

    def get_obs(self):
        # 世界坐标系
        arm_quat = self.shm_state.arm_quat[[1, 2, 3, 0]]  # (w, x, y, z) -> (x, y, z, w)
        if arm_quat[3] < 0.0:  # Enforce quaternion uniqueness
            np.negative(arm_quat, out=arm_quat)  # 确保四元数唯一性
        # 首次获取也是在这里
        obs = {
            
            'arm_pos': self.shm_state.arm_pos.copy(),  # 世界坐标系 - 手臂位置
            'arm_quat': arm_quat,  # 世界坐标系 - 手臂四元数
            'gripper_pos': self.shm_state.gripper_pos.copy(),  # 世界坐标系 - 夹爪位置
        }
        if self.render_images:
            for shm_image in self.shm_images:
                obs[f'{shm_image.camera_name}_image'] = shm_image.data.copy()  # 添加图像数据
        
        # print the current data in mujoco world
        wd_arm_rpy = R.from_quat(obs['arm_quat']).as_euler('xyz', degrees=True)
        print(f"[MujocoEnv-get_obs] mujoco world arm_pos {np.round(obs['arm_pos'], 2)} arm_rot {np.round(wd_arm_rpy, 2)}")
        
        
        return obs  # 返回观测数据

    def step(self, action):
        # Note: We intentionally do not return obs here to prevent the policy from using outdated data
        self.command_queue.put(action)  # 发送动作命令

    def close(self):
        self.shm_state.close()  # 关闭共享内存状态
        self.shm_state.shm.unlink()  # 解除共享内存链接
        if self.render_images:
            for shm_image in self.shm_images:
                shm_image.close()  # 关闭共享内存图像
                shm_image.shm.unlink()  # 解除共享内存链接
                
def rpy_to_quat(rpy):
    return R.from_euler('xyz', rpy).as_quat()


if __name__ == '__main__':
    env = MujocoEnv()  # 创建MuJoCo环境
    # env = MujocoEnv(show_images=True)
    # env = MujocoEnv(render_images=False)
    try:
        while True:
            env.reset()  # 重置环境
            for _ in range(100):
                
                ### setting a rpy to test
                arm_rpy = [np.pi/2, 0, np.pi/2] # home position
                arm_quat = rpy_to_quat(arm_rpy)
                print(f"[main-test] arm_quat {arm_quat}")
                
                action = {
                    'arm_pos': np.array([0.55, 0, 0.4]),
                    'arm_quat': arm_quat,
                    'gripper_pos': np.array([0.8]) 
                }
                
                ## setting a random action to test
                # action = {
                #     
                #     # 'arm_pos': np.array([0.55, 0, 0.4]),
                #     # 'arm_quat': np.random.rand(4),  # 随机生成手臂四元数
                #     # 'gripper_pos': np.random.rand(1),  # 随机生成夹爪位置
                # }
                env.step(action)  # 执行动作
                obs = env.get_obs()  # 获取观测数据
                # print([(k, v.shape) if v.ndim == 3 else (k, v) for (k, v) in obs.items()])  # 打印观测数据
                time.sleep(POLICY_CONTROL_PERIOD)  # 等待控制周期
    finally:
        env.close()  # 关闭环境

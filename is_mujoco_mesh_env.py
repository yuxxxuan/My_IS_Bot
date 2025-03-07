# Author: ZMAI
# Date: 2025-03-07

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
from is_mujoco_env import ShmState, ShmImage, ArmController, MujocoSim

class KinovaMeshMujocoEnv:
    """
    Mujoco环境，包含Kinova Gen3机械臂和poisson_mesh.obj
    """
    def __init__(self, render_images=True, show_viewer=True, show_images=False):
        # 使用新的场景文件
        self.mjcf_path = 'models/kinova_gen3/scene_with_mesh.xml'
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
        renderers = []
        for shm_image in self.shm_images:
            renderers.append(Renderer(model, data, shm_image))
        
        # 渲染循环
        while True:
            for renderer in renderers:
                renderer.render()
            time.sleep(1 / 30)  # 30 FPS
    
    def visualizer_loop(self):
        # 显示图像
        while True:
            for shm_image in self.shm_images:
                if np.any(shm_image.data):
                    cv.imshow(shm_image.camera_name, shm_image.data)
            if cv.waitKey(1) == ord('q'):
                break
        cv.destroyAllWindows()
    
    def reset(self):
        # 重置环境
        self.command_queue.put({'type': 'reset'})
        
        # 等待初始化完成
        while not self.shm_state.initialized[0]:
            time.sleep(0.01)
        
        return self.get_obs()
    
    def get_obs(self):
        # 获取观测
        obs = {
            'arm_pos': np.copy(self.shm_state.arm_pos),
            'arm_quat': np.copy(self.shm_state.arm_quat),
            'gripper_pos': np.copy(self.shm_state.gripper_pos),
        }
        
        if self.render_images:
            for shm_image in self.shm_images:
                obs[f'image/{shm_image.camera_name}'] = np.copy(shm_image.data)
        
        return obs
    
    def step(self, action):
        # 发送动作
        self.command_queue.put({'type': 'action', 'action': action})
        # 注意：我们故意不在这里返回obs，以防止策略使用过时的数据
    
    def close(self):
        # 关闭环境
        self.command_queue.put({'type': 'close'})
        self.shm_state.close()
        if self.render_images:
            for shm_image in self.shm_images:
                shm_image.close()

# 渲染器类，从is_mujoco_env.py复制过来
class Renderer:
    def __init__(self, model, data, shm_image):
        self.model = model
        self.data = data
        self.shm_image = shm_image
        
        # 查找相机ID
        for i in range(model.ncam):
            if model.camera(i).name == shm_image.camera_name:
                self.camera_id = i
                break
        else:
            raise ValueError(f"Camera {shm_image.camera_name} not found")
        
        # 创建渲染上下文
        self.height, self.width = shm_image.data.shape[:2]
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        
        # 创建相机
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.camera.fixedcamid = self.camera_id
        
        # 创建场景
        self.scene = mujoco.MjvScene(model, maxgeom=10000)
        
        # 创建选项
        self.options = mujoco.MjvOption()
        mujoco.mjv_defaultOption(self.options)
    
    def render(self):
        # 更新场景
        mujoco.mjv_updateScene(self.model, self.data, self.options, None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
        
        # 渲染场景
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.context)
        viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        mujoco.mjr_render(viewport, self.scene, self.context)
        
        # 读取像素
        rgb_arr = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(rgb_arr, None, viewport, self.context)
        
        # 翻转图像（OpenGL坐标系与图像坐标系不同）
        rgb_arr = np.flipud(rgb_arr)
        
        # 复制到共享内存
        np.copyto(self.shm_image.data, rgb_arr)
    
    def close(self):
        mujoco.mjr_freeContext(self.context)
        mujoco.mjv_freeScene(self.scene) 
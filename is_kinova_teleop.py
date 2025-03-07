# Author: ZMAI
# Date: 2025-03-06

import logging
import math
import socket
import threading
import time
from queue import Queue
import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from scipy.spatial.transform import Rotation as R
from constants import POLICY_CONTROL_PERIOD

# 设备相机偏移量
DEVICE_CAMERA_OFFSET = np.array([0.0, 0.02, -0.04])  # iPhone 14 Pro
TWO_PI = 2 * math.pi

# 坐标系转换：WebXR到机器人
def convert_webxr_pose(pos, quat):
    # WebXR: +x右, +y上, +z后; 机器人: +x前, +y左, +z上
    pos = np.array([-pos['z'], -pos['x'], pos['y']], dtype=np.float64)
    rot = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])

    # 应用偏移量，使旋转围绕设备中心而不是设备相机
    pos = pos + rot.apply(DEVICE_CAMERA_OFFSET)

    return pos, rot

class WebServer:
    """Web服务器，用于与WebXR客户端通信"""
    def __init__(self, queue):
        self.app = Flask(__name__)
        self.socketio = SocketIO(self.app)
        self.queue = queue

        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.socketio.on('message')
        def handle_message(data):
            # 发送时间戳回去用于RTT计算（5GHz Wi-Fi上的预期RTT为7ms）
            emit('echo', data['timestamp'])

            # 将数据添加到队列中进行处理
            self.queue.put(data)

        # 减少Flask的详细日志输出
        logging.getLogger('werkzeug').setLevel(logging.WARNING)

    def run(self):
        # 获取IP地址
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(('8.8.8.8', 1))
            address = s.getsockname()[0]
        except Exception:
            address = '127.0.0.1'
        finally:
            s.close()
        print(f'启动服务器于 {address}:5000')
        self.socketio.run(self.app, host='0.0.0.0')

class KinovaTeleopController:
    """专门为Kinova Gen3机械臂设计的遥操作控制器"""
    def __init__(self):
        # 遥操作设备ID
        self.primary_device_id = None
        self.enabled_counts = {}

        # 遥操作目标
        self.targets_initialized = False
        self.arm_target_pos = None  # 改为None，等待从观测中获取初始值
        self.arm_target_rot = None
        self.gripper_target_pos = None

        # WebXR参考姿态
        self.arm_xr_ref_pos = None
        self.arm_xr_ref_rot_inv = None

        # 机器人参考姿态
        self.arm_ref_pos = None
        self.arm_ref_rot = None
        self.gripper_ref_pos = None
        
        # 添加初始化标志，确保稳定控制
        self.sent_initial_action = False

    def process_message(self, data):
        """处理来自WebXR客户端的消息"""
        if not self.targets_initialized:
            return

        # 使用设备ID区分不同设备
        device_id = data['device_id']

        # 更新发送此消息的设备的启用计数
        self.enabled_counts[device_id] = self.enabled_counts.get(device_id, 0) + 1 if 'teleop_mode' in data else 0

        # 分配主设备
        if self.enabled_counts[device_id] > 2:
            if self.primary_device_id is None:
                # 注意：我们跳过前2步，因为WebXR姿态更新的延迟高于触摸事件
                self.primary_device_id = device_id
        elif self.enabled_counts[device_id] == 0:
            if device_id == self.primary_device_id:
                self.primary_device_id = None  # 主设备不再启用
                self.arm_xr_ref_pos = None

        # 遥操作已启用
        if self.primary_device_id is not None and 'teleop_mode' in data:
            pos, rot = convert_webxr_pose(data['position'], data['orientation'])

            # 机械臂移动
            if data['teleop_mode'] == 'arm':
                # 存储参考姿态
                if self.arm_xr_ref_pos is None:
                    self.arm_xr_ref_pos = pos
                    self.arm_xr_ref_rot_inv = rot.inv()
                    self.arm_ref_pos = self.arm_target_pos.copy()
                    self.arm_ref_rot = self.arm_target_rot
                    self.gripper_ref_pos = self.gripper_target_pos

                # 位置
                pos_diff = pos - self.arm_xr_ref_pos  # WebXR
                self.arm_target_pos = self.arm_ref_pos + pos_diff

                # 方向
                self.arm_target_rot = (rot * self.arm_xr_ref_rot_inv) * self.arm_ref_rot

                # 夹爪位置
                self.gripper_target_pos = np.clip(self.gripper_ref_pos + data['gripper_delta'], 0.0, 1.0)

    def step(self, obs):
        """根据观测生成动作"""
        # 初始化目标
        if not self.targets_initialized:
            if 'arm_pos' in obs and 'arm_quat' in obs and 'gripper_pos' in obs:
                self.arm_target_pos = obs['arm_pos'].copy()
                self.arm_target_rot = R.from_quat(obs['arm_quat'])
                self.gripper_target_pos = obs['gripper_pos'].copy()
                self.targets_initialized = True
                print("目标位置已初始化")
                
                # 立即生成初始动作，确保机械臂稳定
                self.sent_initial_action = True
                action = {}
                action['arm_pos'] = self.arm_target_pos.copy()
                action['arm_quat'] = self.arm_target_rot.as_quat()
                action['gripper_pos'] = self.gripper_target_pos.copy()
                return action
            return None

        # 生成动作 - 即使没有遥控器输入也保持稳定
        action = {}
        action['arm_pos'] = self.arm_target_pos.copy()
        action['arm_quat'] = self.arm_target_rot.as_quat()
        action['gripper_pos'] = self.gripper_target_pos.copy()
        return action

class KinovaTeleopPolicy:
    """专门为Kinova Gen3机械臂设计的遥操作策略"""
    
    def __init__(self):
        self.web_server_queue = Queue()
        self.teleop_controller = None
        self.teleop_state = None  # 状态: episode_started -> episode_ended -> reset_env
        self.episode_ended = False
        self.last_action_time = time.time()
        self.debug_mode = True  # 调试模式

        # 用于提供WebXR手机Web应用的Web服务器
        server = WebServer(self.web_server_queue)
        threading.Thread(target=server.run, daemon=True).start()

        # 监听线程，用于处理来自WebXR客户端的消息
        threading.Thread(target=self.listener_loop, daemon=True).start()
        
    def reset(self):
        print("正在初始化遥操作策略...")
        self.teleop_controller = KinovaTeleopController()
        self.episode_ended = False
        self.last_action_time = time.time()

        # 等待用户发出已开始记录的信号
        self.teleop_state = None
        while self.teleop_state != 'episode_started':
            time.sleep(0.01)
            
        if self.debug_mode:
            print(f"遥操作控制器状态: primary_device_id={self.teleop_controller.primary_device_id}")
            print(f"targets_initialized={self.teleop_controller.targets_initialized}")
            
        print("遥操作策略初始化完成")
        
    def step(self, obs):
        # 控制频率限制
        current_time = time.time()
        if current_time - self.last_action_time < POLICY_CONTROL_PERIOD:
            return None
        self.last_action_time = current_time
        
        # 确保观测中包含必要的数据
        if 'arm_pos' not in obs or 'arm_quat' not in obs or 'gripper_pos' not in obs:
            if self.debug_mode:
                print(f"警告: 观测数据不完整 - {list(obs.keys())}")
            return None
        
        # 用户结束记录的信号
        if not self.episode_ended and self.teleop_state == 'episode_ended':
            self.episode_ended = True
            return 'end_episode'

        # 用户准备重置环境的信号（在结束记录后）
        if self.teleop_state == 'reset_env':
            return 'reset_env'
            
        # 获取动作
        action = self.teleop_controller.step(obs)
        
        # 调试输出
        if self.debug_mode and isinstance(action, dict):
            print(f"动作: {action.keys()}")
            if 'arm_pos' in action:
                print(f"arm_pos: {action['arm_pos']}")
        
        return action
        
    def listener_loop(self):
        """监听来自WebXR客户端的消息"""
        while True:
            if not self.web_server_queue.empty():
                data = self.web_server_queue.get()

                # 更新状态
                if 'state_update' in data:
                    self.teleop_state = data['state_update']
                    if self.debug_mode:
                        print(f"状态更新: {self.teleop_state}")

                # 处理消息（如果不是过时的）
                elif 1000 * time.time() - data['timestamp'] < 250:  # 250毫秒
                    if self.teleop_controller:
                        self.teleop_controller.process_message(data)
                    elif self.debug_mode:
                        print("警告: teleop_controller未初始化")

            time.sleep(0.001)
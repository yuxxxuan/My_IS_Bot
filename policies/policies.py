# Author: ZMAI  
# Date: 2025-05-04
# Version: 1.1
# policies.py

import logging  # 导入日志模块
import math  # 导入数学模块
import socket  # 导入套接字模块
import threading  # 导入线程模块
import time  # 导入时间模块
from queue import Queue  # 导入队列模块
import cv2 as cv  # 导入OpenCV模块
import numpy as np  # 导入NumPy模块
import zmq  # 导入ZeroMQ模块
from flask import Flask, render_template  # 导入Flask框架
from flask_socketio import SocketIO, emit  # 导入Flask-SocketIO模块
from scipy.spatial.transform import Rotation as R  # 导入旋转变换模块
import os, sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
from configs.constants import POLICY_SERVER_HOST, POLICY_SERVER_PORT, POLICY_IMAGE_WIDTH, POLICY_IMAGE_HEIGHT  # 导入常量
import xml.etree.ElementTree as ET  # 导入XML解析模块


## constants
# 这尼玛其他手机怎么办？没教我怎么算偏移？？？而且，这么小的偏移怎么算的？？？
# [x,y,z] -> 手机相机相对于手机中心的偏移
DEVICE_CAMERA_OFFSET = np.array([0.0, 0.02, -0.04])  # iPhone 14 Pro的设备相机偏移
TWO_PI = 2 * math.pi  # 定义2π

class Policy:
    def reset(self):  # 重置策略状态
        """重置策略状态"""
        raise NotImplementedError  # 抛出未实现异常

    def step(self, obs):  # 根据观测生成动作
        """根据观测生成动作
        Args:
            obs: 观测数据
        Returns:
            动作
        """
        raise NotImplementedError  # 抛出未实现异常

class WebServer:
    def __init__(self, queue):
        """初始化Web服务器
        Args:
            queue: 用于处理消息的队列
        """
        self.app = Flask(__name__)  # 创建Flask应用
        self.socketio = SocketIO(self.app)  # 初始化SocketIO
        self.queue = queue  # 存储消息的队列

        @self.app.route('/')
        def index():
            """主页路由"""
            return render_template('index.html')  # 渲染主页模板

        @self.socketio.on('message')
        def handle_message(data):
            """处理接收到的消息
            Args:
                data: 接收到的数据
            """
            emit('echo', data['timestamp'])  # 发送回显消息
            self.queue.put(data)  # 将数据放入队列中处理

        # 减少Flask日志输出
        logging.getLogger('werkzeug').setLevel(logging.WARNING)

    def run(self):
        """运行Web服务器"""
        # 获取IP地址
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(('8.8.8.8', 1))  # 连接到外部地址
            address = s.getsockname()[0]  # 获取本地IP地址
        except Exception:
            address = '127.0.0.1'  # 默认地址
        finally:
            s.close()
        print(f'[WebServer-run]: Starting server at {address}:5000')  # 打印服务器启动信息
        self.socketio.run(self.app, host='0.0.0.0')  # 启动SocketIO服务器


# Convert coordinate system from WebXR to robot
def convert_webxr_pose(pos, quat):
    """将WebXR坐标系转换为机器人坐标系
    Args:
        pos: 位置字典
        quat: 四元数字典
    Returns:
        转换后的位置和旋转
    """
    '''
    WebXR 的坐标系是右手坐标系，+x 方向向右，+y 方向向上，+z 方向向后。
    机器人坐标系通常是 +x 方向向前，+y 方向向左，+z 方向向上。
    因此，位置的转换规则是：
    x 变为 -z
    y 变为 -x
    z 变为 y
    w 保持不变
    '''
    pos = np.array([-pos['z'], -pos['x'], pos['y']], dtype=np.float64)  # 转换位置
    rot = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])  # 转换四元数

    # 应用偏移，使旋转围绕设备中心而不是设备相机
    pos = pos + rot.apply(DEVICE_CAMERA_OFFSET)  # 应用偏移

    return pos, rot  # 返回转换后的位置和旋转

def model_path_constructor():
    current_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_dir, '..', 'models', 'gen3_2f85.xml')
    model_path = os.path.abspath(model_path) # 绝对路径,可加可不加,效果一样
    return model_path

class TeleopController:
    # only gen3 ->  初始化可注释可不注释
    def __init__(self):
        """初始化遥操作设备
        """
        
        # Teleop device IDs
        self.primary_device_id = None    # 主设备控制手臂或基座
        self.secondary_device_id = None  # 可选的次要设备控制基座
        self.enabled_counts = {}  # 设备启用计数

        # 移动基座位置
        self.base_pose = None
        
        
        ## Teleop目标
        self.targets_initialized = False 
        
        # base
        # self.base_target_pose = None  # 基座目标位置
        
        # arm + gripper
        self.arm_target_pos = None  # 手臂目标位置
        self.arm_target_rot = None  # 手臂目标旋转
        self.gripper_target_pos = None  # 夹爪目标位置

        # WebXR参考姿态
        self.base_xr_ref_pos = None  # 基座WebXR参考位置
        self.base_xr_ref_rot_inv = None  # 基座WebXR参考旋转的逆
        
        self.arm_xr_ref_pos = None  # 手臂WebXR参考位置
        self.arm_xr_ref_rot_inv = None  # 手臂WebXR参考旋转的逆

        # robot参考姿态
        self.arm_ref_pos = None  # 手臂参考位置
        self.arm_ref_rot = None  # 手臂参考旋转
        self.arm_ref_base_pose = None  # arm相对于base
        self.gripper_ref_pos = None  # 夹爪参考位置
        
        self.base2arm_ref_pose = None  # 由世界坐标到arm的相对坐标参考位置，原代码不改动
        mujoco_xml_path = model_path_constructor()
        print(f"[TeleopController-__init__] mujoco_xml_path: {mujoco_xml_path}")
        self._parse_base_link_pos(mujoco_xml_path)
        print(f"[TeleopController-__init__] base2arm_ref_pose: {self.base2arm_ref_pose}")
        
    def _parse_base_link_pos(self, xml_path):
        """从XML中解析gen3/base_link的pos"""
        tree = ET.parse(xml_path)
        root = tree.getroot()
        
        # 遍历worldbody下的body元素
        for body in root.findall("./worldbody/body"):
            if body.get("name") == "gen3/base_link":
                pos_str = body.get("pos")
                self.base2arm_ref_pose = np.array([float(x) for x in pos_str.split()])
                break
        else:
            raise ValueError("XML中未找到body[name='gen3/base_link']")
    def process_message(self, data):
        # only gen3 1
        """处理接收到的消息
        Args:
            data: 接收到的数据
        """
        if not self.targets_initialized:
            return  # 如果目标未初始化，直接返回

        # 使用设备ID区分主设备和次要设备
        device_id = data['device_id']

        # 更新发送此消息的设备的启用计数
        self.enabled_counts[device_id] = self.enabled_counts.get(device_id, 0) + 1 if 'teleop_mode' in data else 0

        # 分配主设备和次要设备
        if self.enabled_counts[device_id] > 2:
            if self.primary_device_id is None and device_id != self.secondary_device_id:
                # 注意：我们跳过前两个步骤，因为WebXR姿态更新的延迟高于触摸事件
                self.primary_device_id = device_id
            elif self.secondary_device_id is None and device_id != self.primary_device_id:
                self.secondary_device_id = device_id
        elif self.enabled_counts[device_id] == 0:
            if device_id == self.primary_device_id:
                self.primary_device_id = None  # 主设备不再启用
                # self.base_xr_ref_pos = None
                self.arm_xr_ref_pos = None
            elif device_id == self.secondary_device_id:
                self.secondary_device_id = None
                # self.base_xr_ref_pos = None

        ## Teleop已启用
        if self.primary_device_id is not None and 'teleop_mode' in data:
            print(f"[TeleopController-process_message] 手机WebXR原始 data: {data}")
            pos, rot = convert_webxr_pose(data['position'], data['orientation'])  # 转换WebXR姿态
            # 2025-03-17 17:57
            print(f"[TeleopController-process_message] 手机WebXR处理后 data: pos: {pos}, rot: {rot.as_quat()}")
            
            # 基座移动
            if data['teleop_mode'] == 'base' or device_id == self.secondary_device_id:  # 注意：次要设备只能控制基座
                # 存储参考姿态
                # if self.base_xr_ref_pos is None:
                #     self.base_ref_pose = self.base_pose.copy()  # 记录基座参考位置
                #     self.base_xr_ref_pos = pos[:2]  # 记录基座WebXR参考位置
                #     self.base_xr_ref_rot_inv = rot.inv()  # 记录基座WebXR参考旋转的逆

                # # 位置
                # self.base_target_pose[:2] = self.base_ref_pose[:2] + (pos[:2] - self.base_xr_ref_pos)  # 更新基座目标位置

                # # 方向
                # base_fwd_vec_rotated = (rot * self.base_xr_ref_rot_inv).apply([1.0, 0.0, 0.0])  # 计算基座前进方向
                # base_target_theta = self.base_ref_pose[2] + math.atan2(base_fwd_vec_rotated[1], base_fwd_vec_rotated[0])  # 更新基座目标角度
                # self.base_target_pose[2] += (base_target_theta - self.base_target_pose[2] + math.pi) % TWO_PI - math.pi  # 解包
                print("ban the base control")

            # 手臂移动
            elif data['teleop_mode'] == 'arm':
                # 存储参考姿态
                if self.arm_xr_ref_pos is None:
                    ### 记录手臂WebXR参考位置! 这个很重要！
                    self.arm_xr_ref_pos = pos  
                    self.arm_xr_ref_rot_inv = rot.inv()  # 记录手臂WebXR参考旋转的逆
                    self.arm_ref_pos = self.arm_target_pos.copy()  # 记录手臂目标位置
                    self.arm_ref_rot = self.arm_target_rot  # 记录手臂目标旋转
                    self.gripper_ref_pos = self.gripper_target_pos  # 记录夹爪参考位置
                    
                    # 这个相对位置在哪有用？是否有其他的潜在风险？ mb
                    # self.arm_ref_base_pose = self.base_pose.copy()  # 记录基座参考位置

                # 绕z轴旋转以在全局坐标系（基座）和局部坐标系（手臂）之间转换
                # base_pose[2] -> 当前 base 的z轴角度
                # 创建旋转对象
                # z_rot = R.from_rotvec(np.array([0.0, 0.0, 1.0]) * self.base_pose[2])  
                # z_rot_inv = z_rot.inv() # 求逆
                # arm_ref_base_pose[2] -> 当前 arm 相对于 base 的z轴角度
                # ref_z_rot = R.from_rotvec(np.array([0.0, 0.0, 1.0]) * self.arm_ref_base_pose[2])  # 手臂参考旋转

                ### 这部分，是遥操作映射的关键！但是我还是不太理解 mb
                # 位置
                #pos -> current pos
                #arm_xr_ref_pos -> last pos
                pos_diff = pos - self.arm_xr_ref_pos  # WebXR位置差
                # 删除base的旋转补偿
                
                factor = 0.5
                self.arm_target_pos = self.arm_ref_pos + pos_diff * factor
                # pos_diff += ref_z_rot.apply(self.arm_ref_pos) - z_rot.apply(self.arm_ref_pos)  # 次要基座控制：补偿基座旋转
                # pos_diff[:2] += self.arm_ref_base_pose[:2] - self.base_pose[:2]  # 次要基座控制：补偿基座平移
                # self.arm_target_pos = self.arm_ref_pos + z_rot_inv.apply(pos_diff)  # 更新手臂目标位置

                # 实时的方向
                # self.arm_target_rot = (z_rot_inv * (rot * self.arm_xr_ref_rot_inv) * ref_z_rot) * self.arm_ref_rot  # 更新手臂目标旋转
                self.arm_target_rot = (rot * self.arm_xr_ref_rot_inv) * self.arm_ref_rot
                
                # 夹爪位置
                self.gripper_target_pos = np.clip(self.gripper_ref_pos + data['gripper_delta'], 0.0, 1.0)  # 更新夹爪目标位置
                

        # Teleop已禁用
        # elif self.primary_device_id is None:
        #     # 更新目标姿态以防止在遥控禁用时基座被推
        #     self.base_target_pose = self.base_pose  # 更新基座目标位置

    def step(self, obs):
        # only gen3 1
        """更新机器人状态
        Args:
            obs: obs 格式？？？ mb 03-18 14:48
        """
        # 直接删除?
        # self.base_pose = obs['base_pose']  # 更新基座位置

        # 初始化目标
        if not self.targets_initialized:
            # base
            # self.base_target_pose = obs['base_pose']  # 初始化基座目标位置
            # arm
            # 初始化这里，是不是有点问题？ 03-18 mb 是的，有问题
            # 当gen3基座位置改变时，obs['arm_pos']的值也会随之改变，因为它是在世界坐标系中表示的。这导致初始目标位置被设置为新的世界坐标位置，而没有考虑到gen3基座位置的变化
            
            # base_pos 是根据 xml里面，gen3/base_link 的 pos进行配置, 应当与gen3/base_link 的 pos一致
            base_pos = self.base2arm_ref_pose # 写了个一次性获取 gen3/base_link pos
            self.arm_target_pos = obs['arm_pos'] - base_pos # 初始化手臂目标位置
            self.arm_target_rot = R.from_quat(obs['arm_quat'])  # 初始化手臂目标旋转
            
            # self.arm_target_pos = np.array([0.456, 0.0, 0.434]) # 初始化手臂目标位置
            # self.arm_target_rot = R.from_quat([0.0, 0.0, 0.0, 1.0])  # 初始化手臂目标旋转
            
            # gripper
            self.gripper_target_pos = obs['gripper_pos']  # 初始化夹爪目标位置
            self.targets_initialized = True  # 标记目标已初始化
            
            print(f"[TeleopController-step] init arm_pos: {self.arm_target_pos} arm_quat: {self.arm_target_rot.as_quat()}")

        # 如果遥控未启用，则返回无动作
        if self.primary_device_id is None:
            return None

        # 获取最新的遥控命令
        arm_quat = self.arm_target_rot.as_quat()  # 获取手臂目标旋转的四元数
        if arm_quat[3] < 0.0:  # 确保四元数唯一性
            np.negative(arm_quat, out=arm_quat)  # 反转四元数
        action = {
            # 'base_pose': self.base_target_pose.copy(),  # 返回基座目标位置
            'arm_pos': self.arm_target_pos.copy(),  # 返回手臂目标位置
            'arm_quat': arm_quat,  # 返回手臂目标旋转
            'gripper_pos': self.gripper_target_pos.copy(),  # 返回夹爪目标位置
        }
        print(f"-----------------------------------------\n"
              f"[TeleopController-step] action: {action}\n"
              f"-----------------------------------------")
        return action  # 返回动作

# Teleop using WebXR phone web app
# main.py -> 调用 -> TeleopPolicy -> 创建 -> TeleopController -> 创建 -> WebServer -> 创建 -> WebXRListener
class TeleopPolicy(Policy):
    def __init__(self):
        """初始化遥控策略
        """
        self.web_server_queue = Queue()  # 创建消息队列
        self.teleop_controller = None  # 初始化遥控控制器
        self.teleop_state = None  # 状态：episode_started -> episode_ended -> reset_env
        self.episode_ended = False  # 记录是否结束

        # Web服务器，用于提供WebXR手机Web应用
        server = WebServer(self.web_server_queue)  # 创建Web服务器
        threading.Thread(target=server.run, daemon=True).start()  # 启动Web服务器线程

        # 监听线程，处理来自WebXR客户端的消息
        threading.Thread(target=self.listener_loop, daemon=True).start()  # 启动监听线程
        # print("[TeleopPolicy-init] Finished")
    def reset(self):
        """重置遥控策略
        """
        self.teleop_controller = TeleopController()  # 创建遥控控制器
        self.episode_ended = False  # 重置结束状态

        # 等待用户信号，表示episode已开始
        self.teleop_state = None
        while self.teleop_state != 'episode_started':
            time.sleep(0.01)  # 等待
            
        print("[TeleopPolicy-reset] Finished")

    def step(self, obs):
        """执行一步策略
        Args:
            obs: 观测数据
        Returns:
            动作或状态信号
        """
        # 信号用户已结束episode
        if not self.episode_ended and self.teleop_state == 'episode_ended':
            self.episode_ended = True  # 标记为结束
            return 'end_episode'  # 返回结束信号

        # 信号用户准备重置环境（在结束episode后）
        if self.teleop_state == 'reset_env':
            return 'reset_env'  # 返回重置信号

        return self._step(obs)  # 执行一步策略

    def _step(self, obs):
        """执行内部步骤
        Args:
            obs: 观测数据
        Returns:
            动作
        """
        return self.teleop_controller.step(obs)  # 调用遥操作的步骤

    def listener_loop(self):
        """监听消息循环
        """
        while True:
            if not self.web_server_queue.empty():
                
                ### 获取消息! 这一步，是将信号，从web端传送到了这里！ mb
                data = self.web_server_queue.get()  

                # 更新状态
                if 'state_update' in data:
                    self.teleop_state = data['state_update']  # 更新状态

                # 处理消息（如果未过期）
                elif 1000 * time.time() - data['timestamp'] < 250:  # 250 ms
                    # print("[TeleopPolicy-listener_loop] Get data")
                    self._process_message(data)  # 处理消息

            time.sleep(0.001)  # 等待

    def _process_message(self, data):
        """处理接收到的消息
        Args:
            data: 接收到的数据
        """
        self.teleop_controller.process_message(data)  # 调用遥控控制器处理消息
        # print("[TeleopPolicy-_process_message] Finished")
        

# Execute policy running on remote server(云端部署的模型服务)
class RemotePolicy(TeleopPolicy):
    def __init__(self):
        """初始化远程策略
        """
        super().__init__()  # 调用父类构造函数

        # 使用手机作为策略执行的启用设备
        self.enabled = False  # 初始化为禁用

        # 连接到策略服务器
        context = zmq.Context()  # 创建ZeroMQ上下文
        self.socket = context.socket(zmq.REQ)  # 创建请求套接字
        self.socket.connect(f'tcp://{POLICY_SERVER_HOST}:{POLICY_SERVER_PORT}')  # 连接到策略服务器
        print(f'Connected to policy server at {POLICY_SERVER_HOST}:{POLICY_SERVER_PORT}')  # 打印连接信息

    def reset(self):
        """重置远程策略
        """
        super().reset()  # 调用父类重置方法

        # 检查与策略服务器的连接并重置策略
        default_timeout = self.socket.getsockopt(zmq.RCVTIMEO)  # 获取默认超时
        self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 临时设置超时为1000 ms
        self.socket.send_pyobj({'reset': True})  # 发送重置请求
        try:
            self.socket.recv_pyobj()  # 接收响应
        except zmq.error.Again as e:
            raise Exception('Could not communicate with policy server') from e  # 抛出异常
        self.socket.setsockopt(zmq.RCVTIMEO, default_timeout)  # 恢复默认超时

        # 禁用策略执行，直到用户按下屏幕
        self.enabled = False  # 注意：设置为True以在没有手机的情况下运行

    def _step(self, obs):
        """执行远程步骤
        Args:
            obs: 观测数据
        Returns:
            动作
        """
        # 如果episode已结束，返回遥控命令
        if self.episode_ended:
            return self.teleop_controller.step(obs)

        # 如果机器人未启用，则返回无动作
        if not self.enabled:
            return None

        # 编码图像
        encoded_obs = {}
        for k, v in obs.items():
            if v.ndim == 3:
                # 调整图像大小以符合策略服务器的期望分辨率
                v = cv.resize(v, (POLICY_IMAGE_WIDTH, POLICY_IMAGE_HEIGHT))

                # 将图像编码为JPEG
                _, v = cv.imencode('.jpg', v)  # 注意：将RGB解释为BGR
                encoded_obs[k] = v  # 存储编码后的图像
            else:
                encoded_obs[k] = v  # 存储其他观测数据

        # 发送观测数据到策略服务器
        req = {'obs': encoded_obs}  # 创建请求
        self.socket.send_pyobj(req)  # 发送请求

        # 从策略服务器获取动作
        rep = self.socket.recv_pyobj()  # 接收响应
        action = rep['action']  # 获取动作

        return action  # 返回动作

    def _process_message(self, data):
        """处理接收到的消息
        Args:
            data: 接收到的数据
        """
        if self.episode_ended:
            # 如果episode已结束，运行遥控控制器
            self.teleop_controller.process_message(data)
        else:
            # 如果用户按下屏幕，则启用策略执行
            self.enabled = 'teleop_mode' in data

if __name__ == '__main__':
    # WebServer(Queue()).run(); time.sleep(1000)
    # WebXRListener(); time.sleep(1000)
    from configs.constants import POLICY_CONTROL_PERIOD
    obs = {
        # 'base_pose': np.zeros(3),  # 基座位置
        'arm_pos': np.zeros(3),  # 手臂位置
        'arm_quat': np.array([0.0, 0.0, 0.0, 1.0]),  # 手臂四元数
        'gripper_pos': np.zeros(1),  # 夹爪位置
        
        'base_image': np.zeros((640, 360, 3)),  # 基座图像  这个在哪用了？？？mb
        'wrist_image': np.zeros((640, 480, 3)),  # 手腕图像
    }
    policy = TeleopPolicy()  # 创建遥控策略
    # policy = RemotePolicy()  # 创建远程策略
    while True:
        policy.reset()  # 重置策略
        for _ in range(100):
            print(policy.step(obs))  # 打印策略输出
            time.sleep(POLICY_CONTROL_PERIOD)  # 等待控制周期
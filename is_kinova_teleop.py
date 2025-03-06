# Author: ZMAI
# Date: 2025-03-06

import time
import numpy as np
from policies import TeleopPolicy, convert_webxr_pose
from constants import POLICY_CONTROL_PERIOD
from scipy.spatial.transform import Rotation as R
import math

class KinovaTeleopPolicy(TeleopPolicy):
    """专门为Kinova Gen3机械臂设计的遥操作策略"""
    
    def __init__(self):
        super().__init__()
        self.last_action_time = time.time()
        # 添加调试标志
        self.debug_mode = True
        
    def reset(self):
        print("正在初始化遥操作策略...")
        super().reset()
        self.last_action_time = time.time()
        
        # 确保初始化完成后打印状态
        if self.debug_mode and hasattr(self, 'teleop_controller'):
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
            
        # 确保teleop_controller已初始化
        if not hasattr(self, 'teleop_controller') or self.teleop_controller is None:
            if self.debug_mode:
                print("警告: teleop_controller未初始化")
            return None
            
        # 更新机械臂状态 - 这一步很关键，确保控制器知道当前机械臂位置
        if hasattr(self.teleop_controller, 'base_pose') and self.teleop_controller.base_pose is None:
            self.teleop_controller.base_pose = np.zeros(3)  # 只有机械臂，没有移动基座
            
        # 初始化目标位置
        if not self.teleop_controller.targets_initialized:
            self.teleop_controller.targets_initialized = True
            self.teleop_controller.base_target_pose = np.zeros(3)
            self.teleop_controller.arm_target_pos = obs['arm_pos'].copy()
            self.teleop_controller.arm_target_rot = R.from_quat(obs['arm_quat'])
            self.teleop_controller.gripper_target_pos = obs['gripper_pos'].copy()
            if self.debug_mode:
                print(f"初始化目标位置: arm_pos={self.teleop_controller.arm_target_pos}")
        
        # 获取基础动作
        action = super().step(obs)
        
        # 调试输出
        if self.debug_mode and isinstance(action, dict):
            print(f"动作: {action.keys()}")
            if 'arm_pos' in action:
                print(f"arm_pos: {action['arm_pos']}")
        
        # 如果是特殊命令，直接返回
        if action in ['end_episode', 'reset_env']:
            return action
            
        return action
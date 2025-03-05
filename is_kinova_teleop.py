# Author: ZMAI
# Date: 2025-03-06

import time
import numpy as np
from policies import TeleopPolicy
from constants import POLICY_CONTROL_PERIOD

class KinovaTeleopPolicy(TeleopPolicy):
    """专门为Kinova Gen3机械臂设计的遥操作策略"""
    
    def __init__(self):
        super().__init__()
        self.last_action_time = time.time()
        
    def reset(self):
        super().reset()
        self.last_action_time = time.time()
        
    def step(self, obs):
        # 控制频率限制
        current_time = time.time()
        if current_time - self.last_action_time < POLICY_CONTROL_PERIOD:
            return None
        self.last_action_time = current_time
        
        # 获取遥操作控制器的状态
        if not self.teleop_controller.targets_initialized:
            # 初始化目标位置为当前位置
            if 'arm_pos' in obs and 'arm_quat' in obs:
                self.teleop_controller.arm_target_pos = obs['arm_pos'].copy()
                self.teleop_controller.arm_target_rot = obs['arm_quat'].copy()
                self.teleop_controller.gripper_target_pos = obs['gripper_pos'].copy()
                self.teleop_controller.targets_initialized = True
            return None
            
        # 检查是否有遥操作设备连接
        if self.teleop_controller.primary_device_id is None:
            return None
            
        # 构建动作
        action = {
            'arm_pos': self.teleop_controller.arm_target_pos.copy(),
            'arm_quat': self.teleop_controller.arm_target_rot.copy(),
            'gripper_pos': self.teleop_controller.gripper_target_pos.copy(),
        }
        
        # 处理特殊命令
        if self.teleop_controller.command == 'end_episode':
            return 'end_episode'
        elif self.teleop_controller.command == 'reset_env':
            return 'reset_env'
            
        return action
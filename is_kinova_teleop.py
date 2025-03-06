# Author: ZMAI
# Date: 2025-03-06

import time
import numpy as np
from policies import TeleopPolicy
from constants import POLICY_CONTROL_PERIOD
from scipy.spatial.transform import Rotation as R

class KinovaTeleopPolicy(TeleopPolicy):
    """专门为Kinova Gen3机械臂设计的遥操作策略"""
    
    def __init__(self):
        super().__init__()
        self.last_action_time = time.time()
        
    def reset(self):
        print("正在初始化遥操作策略...")
        super().reset()
        self.last_action_time = time.time()
        print("遥操作策略初始化完成")
        
    def step(self, obs):
        # 控制频率限制
        current_time = time.time()
        if current_time - self.last_action_time < POLICY_CONTROL_PERIOD:
            return None
        self.last_action_time = current_time
        
        # 获取基础动作
        action = super().step(obs)
        
        # 如果是特殊命令，直接返回
        if action in ['end_episode', 'reset_env']:
            return action
            
        # 如果没有动作，检查是否可以初始化目标位置
        if action is None and hasattr(self.teleop_controller, 'targets_initialized'):
            if not self.teleop_controller.targets_initialized:
                # 初始化目标位置为当前位置
                if 'arm_pos' in obs and 'arm_quat' in obs:
                    self.teleop_controller.arm_target_pos = obs['arm_pos'].copy()
                    self.teleop_controller.arm_target_rot = R.from_quat(obs['arm_quat'])
                    self.teleop_controller.gripper_target_pos = obs['gripper_pos'].copy()
                    self.teleop_controller.targets_initialized = True
        
        return action
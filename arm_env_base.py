# Author: ZMAI
# Date: 2025-03-06

import time
import numpy as np
from abc import ABC, abstractmethod

class ArmEnvBase(ABC):
    """机械臂环境基类，定义通用接口"""
    
    @abstractmethod
    def reset(self):
        """重置环境"""
        pass
    
    @abstractmethod
    def get_obs(self):
        """获取观测"""
        pass
    
    @abstractmethod
    def step(self, action):
        """执行动作"""
        pass
    
    @abstractmethod
    def close(self):
        """关闭环境"""
        pass
    
    def process_arm_action(self, action):
        """处理机械臂动作，移除底盘相关部分"""
        if action is None:
            return None
            
        arm_action = {}
        if 'arm_pos' in action:
            arm_action['arm_pos'] = action['arm_pos']
        if 'arm_quat' in action:
            arm_action['arm_quat'] = action['arm_quat']
        if 'gripper_pos' in action:
            arm_action['gripper_pos'] = action['gripper_pos']
            
        return arm_action 
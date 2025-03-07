# Author: ZMAI
# Date: 2025-03-06

import time
import numpy as np
from constants import POLICY_CONTROL_PERIOD
from arm_env_base import ArmEnvBase

# 条件导入，只在使用真实环境时才导入
try:
    from kinova import TorqueControlledArm
    HAS_KINOVA_API = True
except ImportError:
    from mock_kinova import MockTorqueControlledArm as TorqueControlledArm
    HAS_KINOVA_API = False
    print("警告: 未找到Kinova API，只能使用模拟环境")

class KinovaRealEnv(ArmEnvBase):
    def __init__(self):
        self.arm = TorqueControlledArm()
        self.arm.init_cyclic(self._control_callback)
        self.last_obs = None
        
    def _control_callback(self, command):
        if command is not None:
            if 'arm_pos' in command and 'arm_quat' in command:
                self.arm.move_to_pose(command['arm_pos'], command['arm_quat'])
            if 'gripper_pos' in command:
                self.arm.set_gripper_position(command['gripper_pos'])
    
    def reset(self):
        self.arm.move_to_home_position()
        self.last_obs = self.get_obs()
        return self.last_obs
    
    def get_obs(self):
        arm_pos, arm_quat = self.arm.get_end_effector_pose()
        gripper_pos = self.arm.get_gripper_position()
        obs = {
            'arm_pos': arm_pos,
            'arm_quat': arm_quat,
            'gripper_pos': gripper_pos,
        }
        self.last_obs = obs
        return obs
    
    def step(self, action):
        # 使用基类的处理方法过滤掉底盘相关动作
        arm_action = self.process_arm_action(action)
        self._control_callback(arm_action)
        return self.get_obs()
    
    def close(self):
        self.arm.disconnect()

if __name__ == '__main__':
    # 测试真实环境
    if not HAS_KINOVA_API:
        print("错误: 无法测试真实环境，未找到Kinova API")
        exit(1)
        
    env = KinovaRealEnv()
    try:
        env.reset()
        print("机械臂已重置到初始位置")
        print("按Ctrl+C退出...")
        
        while True:
            obs = env.get_obs()
            print(f"当前状态: {obs}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("用户中断，正在关闭...")
    finally:
        env.close()
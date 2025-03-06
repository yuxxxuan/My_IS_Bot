"""
测试Kinova机械臂遥操作功能
"""
import time
import numpy as np
from is_kinova_teleop import KinovaTeleopPolicy
from constants import POLICY_CONTROL_PERIOD

# 创建一个简单的模拟环境
class MockEnv:
    def __init__(self):
        self.arm_pos = np.zeros(3)
        self.arm_quat = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w
        self.gripper_pos = np.array([0.0])
        
    def get_obs(self):
        return {
            'arm_pos': self.arm_pos.copy(),
            'arm_quat': self.arm_quat.copy(),
            'gripper_pos': self.gripper_pos.copy(),
        }
        
    def step(self, action):
        if action is None:
            return
            
        if isinstance(action, dict):
            if 'arm_pos' in action:
                print(f"机械臂移动到: {action['arm_pos']}")
                self.arm_pos = action['arm_pos'].copy()
            if 'arm_quat' in action:
                print(f"机械臂旋转到: {action['arm_quat']}")
                self.arm_quat = action['arm_quat'].copy()
            if 'gripper_pos' in action:
                print(f"夹爪位置: {action['gripper_pos']}")
                self.gripper_pos = action['gripper_pos'].copy()

# 主测试函数
def main():
    env = MockEnv()
    policy = KinovaTeleopPolicy()
    
    print("初始化策略...")
    policy.reset()
    print("请在手机上连接到服务器并点击'开始记录'")
    
    try:
        while True:
            obs = env.get_obs()
            action = policy.step(obs)
            
            if action == 'end_episode':
                print("记录结束")
            elif action == 'reset_env':
                print("重置环境")
                break
            elif isinstance(action, dict):
                env.step(action)
                
            time.sleep(POLICY_CONTROL_PERIOD)
    except KeyboardInterrupt:
        print("用户中断，退出测试")

if __name__ == "__main__":
    main()
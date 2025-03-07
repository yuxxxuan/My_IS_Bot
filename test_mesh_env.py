#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试带有mesh的Kinova Gen3环境
"""

import time
import numpy as np
from is_mujoco_mesh_env import KinovaMeshMujocoEnv

def main():
    # 创建环境
    print("创建带有mesh的Kinova Gen3环境...")
    env = KinovaMeshMujocoEnv(show_images=True)
    
    # 重置环境
    print("重置环境...")
    obs = env.reset()
    
    # 打印观测
    print("观测:")
    for key, value in obs.items():
        if key.startswith('image/'):
            print(f"  {key}: 图像形状 {value.shape}")
        else:
            print(f"  {key}: {value}")
    
    # 简单的控制循环
    print("开始控制循环...")
    try:
        for i in range(100):
            # 创建一个简单的动作：移动机械臂并控制夹爪
            action = {
                'type': 'arm_delta',
                'arm_delta': np.array([0.0, 0.0, 0.001]),  # 向上移动
                'gripper_delta': 0.0 if i % 20 < 10 else 1.0  # 周期性开关夹爪
            }
            
            # 执行动作
            env.step(action)
            
            # 等待一段时间
            time.sleep(0.1)
            
            # 每10步打印一次状态
            if i % 10 == 0:
                obs = env.get_obs()
                print(f"步骤 {i}:")
                print(f"  机械臂位置: {obs['arm_pos']}")
                print(f"  夹爪位置: {obs['gripper_pos']}")
    
    except KeyboardInterrupt:
        print("用户中断")
    
    finally:
        # 关闭环境
        print("关闭环境...")
        env.close()

if __name__ == "__main__":
    main() 
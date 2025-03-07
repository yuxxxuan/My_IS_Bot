#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
主函数：运行带有Kinova Gen3机械臂和poisson_mesh.obj的仿真环境
"""

import os
import time
import numpy as np
import argparse
from is_mujoco_mesh_env import KinovaMeshMujocoEnv

def should_save_episode(writer):
    """判断是否应该保存当前回合的数据"""
    if writer is None:
        return False
    
    # 这里可以添加更多的保存条件
    return True

def run_episode(env, policy, writer=None):
    """运行一个回合"""
    # 重置环境
    obs = env.reset()
    
    # 记录初始状态
    if writer is not None:
        writer.add_step(obs, None)
    
    # 运行回合
    done = False
    step = 0
    
    while not done:
        # 获取动作
        action = policy.get_action(obs)
        
        # 执行动作
        env.step(action)
        
        # 获取新的观测
        new_obs = env.get_obs()
        
        # 记录数据
        if writer is not None:
            writer.add_step(new_obs, action)
        
        # 更新状态
        obs = new_obs
        step += 1
        
        # 检查是否结束
        if step >= 1000:  # 设置最大步数
            done = True
    
    # 保存回合数据
    if writer is not None and should_save_episode(writer):
        writer.save_episode()
    
    return step

def main(args):
    """主函数"""
    # 创建环境
    print(f"正在创建带有mesh的模拟环境，模型路径: models/kinova_gen3/scene_with_mesh.xml")
    # 检查模型文件是否存在
    if not os.path.exists('models/kinova_gen3/scene_with_mesh.xml'):
        print(f"警告: 模型文件不存在，请确保路径正确")
    env = KinovaMeshMujocoEnv(show_images=args.show_images)
    
    # 创建策略
    if args.policy == 'random':
        from policies.random_policy import RandomPolicy
        policy = RandomPolicy()
    elif args.policy == 'teleop':
        from policies.teleop_policy import TeleopPolicy
        policy = TeleopPolicy()
    else:
        raise ValueError(f"未知策略: {args.policy}")
    
    # 创建数据记录器
    writer = None
    if args.record:
        from data_writer import DataWriter
        os.makedirs(args.record_dir, exist_ok=True)
        writer = DataWriter(args.record_dir)
    
    try:
        # 运行回合
        print(f"开始运行回合，策略: {args.policy}")
        steps = run_episode(env, policy, writer)
        print(f"回合结束，总步数: {steps}")
    
    except KeyboardInterrupt:
        print("用户中断")
    
    finally:
        # 关闭环境
        env.close()
        
        # 关闭数据记录器
        if writer is not None:
            writer.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='运行带有mesh的Kinova Gen3仿真环境')
    parser.add_argument('--show-images', action='store_true', help='显示相机图像')
    parser.add_argument('--policy', type=str, default='random', choices=['random', 'teleop'], help='使用的策略')
    parser.add_argument('--record', action='store_true', help='记录数据')
    parser.add_argument('--record-dir', type=str, default='data/mesh_teleop', help='记录数据的目录')
    args = parser.parse_args()
    
    main(args) 
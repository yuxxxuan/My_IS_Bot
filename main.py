import argparse
import time
from itertools import count
from configs.constants import POLICY_CONTROL_PERIOD  # 从常量模块导入控制周期
from collector.episode_storage import EpisodeWriter    # 数据存储模块
from policies import TeleopPolicy, RemotePolicy  # 策略控制模块
import numpy as np
from policies.openvla_policies import OpenVLAPolicy  # 导入新的OpenVLAPolicy策略模块

def args_parser_setting():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true')
    parser.add_argument('--teleop', action='store_true')
    parser.add_argument('--save', action='store_true')
    parser.add_argument('--openvla', action='store_true')
    parser.add_argument('--output-dir', default='data/demo_0505')
    return parser.parse_args()
    
def should_save_episode(writer):
    if len(writer) == 0:
        print('Discarding empty episode')
        return False

    # Prompt user whether to save episode
    while True:
        user_input = input('Save episode (y/n)? ').strip().lower()
        if user_input == 'y':
            return True
        if user_input == 'n':
            print('Discarding episode')
            return False
        print('Invalid response')

def run_episode(env, policy, writer=None):
    # Reset the env
    print('Resetting env...')
    env.reset()
    print('Env has been reset')

    # Wait for user to press "Start episode"
    print('Press "Start episode" in the web app when ready to start new episode')
    policy.reset()
    print('Starting new episode')

    episode_ended = False
    start_time = time.time()
    for step_idx in count():
        # Enforce desired control freq
        step_end_time = start_time + step_idx * POLICY_CONTROL_PERIOD
        while time.time() < step_end_time:
            time.sleep(0.0001)

        # Get latest observation
        obs = env.get_obs()
        # print('[main-run_episode]: env.get_obs() Get!')

        # Get action
        action = policy.step(obs)

        # No action if teleop not enabled
        if action is None:
            continue

        # Execute valid action on robot
        if isinstance(action, dict):
            env.step(action)

            if writer is not None and not episode_ended:
                # Record executed action
                writer.step(obs, action)
                
        # Episode ended
        elif isinstance(action, str) and action == 'end_episode':  # 先检查action是否是字符串
            episode_ended = True
            print('Episode ended')

            if writer is not None and should_save_episode(writer):
                # Save to disk in background thread
                writer.flush_async()

            print('Teleop is now active. Press "Reset env" in the web app when ready to proceed.')

        # Ready for env reset
        elif isinstance(action, str) and action == 'reset_env':  # 先检查action是否是字符串
            break

        # # Episode ended
        # elif not episode_ended and action == 'end_episode':
        #     episode_ended = True
        #     print('Episode ended')

        #     if writer is not None and should_save_episode(writer):
        #         # Save to disk in background thread
        #         writer.flush_async()

        #     print('Teleop is now active. Press "Reset env" in the web app when ready to proceed.')

        # # Ready for env reset
        # elif action == 'reset_env':
        #     break

    if writer is not None:
        # Wait for writer to finish saving to disk
        writer.wait_for_flush()

def main(args):
    # Create env
    if args.sim:
        from mujoco_env import MujocoEnv # 动态导入仿真环境
        if args.teleop :
            env = MujocoEnv(show_images=True)
        else:
            env = MujocoEnv(offscreen=True,show_images=True)
    else: # 导入真实环境配置
        from real_env import RealEnv
        env = RealEnv()

    # Create policy / 创建远程连接策略
    if args.teleop:
        policy = TeleopPolicy() # 创建遥控操作策略
    elif args.openvla:
    # 使用新的OpenVLAPolicy策略
        policy = OpenVLAPolicy(server_url="http://192.168.3.101:9000",env=env)  # API地址
    else:
        policy = RemotePolicy() # 创建远程连接策略？ 为了训练？


    try:
        while True: # 持续运行循环
            print("args.save: ", args.save, "args.output_dir: ", args.output_dir)
            writer = EpisodeWriter(args.output_dir) if args.save else None
            
            # 循环运行在这里!
            run_episode(env, policy, writer)
    finally:
        env.close() # 关闭环境

if __name__ == '__main__':
    args = args_parser_setting()
    main(args)

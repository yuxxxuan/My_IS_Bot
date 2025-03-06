# Author: ZMAI
# Date: 2025-03-06

import argparse
import time
from itertools import count
from constants import POLICY_CONTROL_PERIOD
from episode_storage import EpisodeWriter
from is_kinova_teleop import KinovaTeleopPolicy

def should_save_episode(writer):
    if len(writer) == 0:
        print('丢弃空记录')
        return False

    # 提示用户是否保存记录
    while True:
        user_input = input('保存记录 (y/n)? ').strip().lower()
        if user_input == 'y':
            return True
        if user_input == 'n':
            print('丢弃记录')
            return False
        print('无效输入')

def run_episode(env, policy, writer=None):
    # 重置环境
    print('重置环境...')
    env.reset()
    print('环境已重置')

    # 等待用户准备开始
    print('准备好后，请按"开始记录"')
    policy.reset()
    print('开始新记录')

    episode_ended = False
    start_time = time.time()
    for step_idx in count():
        # 控制频率
        step_end_time = start_time + step_idx * POLICY_CONTROL_PERIOD
        while time.time() < step_end_time:
            time.sleep(0.0001)

        # 获取最新观测
        obs = env.get_obs()

        # 获取动作
        action = policy.step(obs)

        # 如果遥操作未启用，则不执行动作
        if action is None:
            continue

        # 执行有效动作
        if isinstance(action, dict):
            env.step(action)

            if writer is not None and not episode_ended:
                # 记录执行的动作
                writer.step(obs, action)

        # 记录结束
        elif not episode_ended and action == 'end_episode':
            episode_ended = True
            print('记录结束')

            if writer is not None and should_save_episode(writer):
                # 后台线程保存到磁盘
                writer.flush_async()

            print('遥操作现在处于活动状态。准备好后，请按"重置环境"')

        # 准备重置环境
        elif action == 'reset_env':
            break

    if writer is not None:
        # 等待写入器完成保存
        writer.wait_for_flush()

def main(args):
    # 创建环境
    if args.sim:
        from is_mujoco_env import KinovaMujocoEnv
        env = KinovaMujocoEnv(show_images=args.show_images)
    else:
        from is_real_env import KinovaRealEnv, HAS_KINOVA_API
        if not HAS_KINOVA_API:
            raise ImportError("无法创建真实环境：未找到Kinova API。请安装kortex_api或使用--sim参数运行模拟环境。")
        env = KinovaRealEnv()

    # 创建策略
    policy = KinovaTeleopPolicy()

    try:
        while True:
            writer = EpisodeWriter(args.output_dir) if args.save else None
            run_episode(env, policy, writer)
    finally:
        env.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='使用模拟环境')
    parser.add_argument('--save', action='store_true', help='保存记录')
    parser.add_argument('--show-images', action='store_true', help='显示相机图像')
    parser.add_argument('--output-dir', default='data/kinova-demos', help='记录保存目录')
    main(parser.parse_args())
# Author: ZMAI
# Date: 2025-03-06

import time
from itertools import count
from constants import POLICY_CONTROL_PERIOD

def should_save_episode(writer, lang='en'):
    """通用的记录保存询问函数，支持中英文"""
    if len(writer) == 0:
        print('Discarding empty episode' if lang == 'en' else '丢弃空记录')
        return False

    # 提示用户是否保存记录
    while True:
        prompt = 'Save episode (y/n)? ' if lang == 'en' else '保存记录 (y/n)? '
        user_input = input(prompt).strip().lower()
        if user_input == 'y':
            return True
        if user_input == 'n':
            print('Discarding episode' if lang == 'en' else '丢弃记录')
            return False
        print('Invalid response' if lang == 'en' else '无效输入')

def run_episode_base(env, policy, writer=None, lang='en', extra_delay=0.0):
    """通用的运行记录函数，支持中英文和额外延迟"""
    # 重置环境
    print('Resetting env...' if lang == 'en' else '重置环境...')
    env.reset()
    print('Env has been reset' if lang == 'en' else '环境已重置')
    
    if extra_delay > 0:
        time.sleep(extra_delay)  # 添加额外延迟确保初始化完成

    # 等待用户准备开始
    start_msg = 'Press "Start episode" in the web app when ready to start new episode' if lang == 'en' else '准备好后，请按"开始记录"'
    print(start_msg)
    policy.reset()
    print('Starting new episode' if lang == 'en' else '开始新记录')

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
            print('Episode ended' if lang == 'en' else '记录结束')

            if writer is not None and should_save_episode(writer, lang):
                # 后台线程保存到磁盘
                writer.flush_async()

            teleop_msg = 'Teleop is now active. Press "Reset env" in the web app when ready to proceed.' if lang == 'en' else '遥操作现在处于活动状态。准备好后，请按"重置环境"'
            print(teleop_msg)

        # 准备重置环境
        elif action == 'reset_env':
            break

    if writer is not None:
        # 等待写入器完成保存
        writer.wait_for_flush() 
import argparse
import time
from itertools import count
from pathlib import Path
import cv2 as cv
from constants import POLICY_CONTROL_PERIOD
from episode_storage import EpisodeReader
from mujoco_env import MujocoEnv

time_per_segment = 0.3
def replay_episode(env, episode_dir, show_images=False, execute_obs=False, sim_showing=False):
    """
    播放指定目录中的回合数据。

    参数:
    env -- Mujoco环境实例
    episode_dir -- 回合数据目录
    show_images -- 是否显示图像观察 (默认: False)
    execute_obs -- 是否执行观察 (默认: False)

    返回:
    无
    异常:
    如果无法加载回合数据，将引发 IOError。
    """
    # 重置环境
    env.reset()

    # 加载回合数据
    reader = EpisodeReader(episode_dir)
    print(f'Loaded episode from {episode_dir}')

    start_time = time.time()
    current_segment = 0
    
    for step_idx, (obs, action) in enumerate(zip(reader.observations, reader.actions)):
        # 计算经过的时间
        elapsed_time = time.time() - start_time
        
        # simulation grounding change here!
        if sim_showing:
            if elapsed_time >= (current_segment + 1) * time_per_segment:
                current_segment += 1
                env.randomize_environment()
                print(f"Switching environment at {elapsed_time:.1f}s (Segment {current_segment})")
        
        
        # 强制执行所需的控制频率
        step_end_time = start_time + step_idx * POLICY_CONTROL_PERIOD
        while time.time() < step_end_time:
            time.sleep(0.0001)

        # 显示图像观察
        if show_images:
            window_idx = 0
            for k, v in obs.items():
                if v.ndim == 3:
                    cv.imshow(k, cv.cvtColor(v, cv.COLOR_RGB2BGR))
                    cv.moveWindow(k, 640 * window_idx, 0)
                    window_idx += 1
            cv.waitKey(1)

        # 在环境中执行动作
        if execute_obs:
            env.step(obs)
        else:
            env.step(action)

def main(args):
    """
    主函数，解析命令行参数并启动回合重放。

    参数:
    args -- 命令行参数对象
    """
    # 创建环境
    if args.sim:
        env = MujocoEnv(render_images=False)
    else:
        from real_env import RealEnv
        env = RealEnv()

    try:
        episode_dirs = sorted([child for child in Path(args.input_dir).iterdir() if child.is_dir()])
        for episode_dir in episode_dirs:
            # here!mb
            replay_episode(env, episode_dir, show_images=args.show_images, execute_obs=args.execute_obs, sim_showing=args.sim_showing)
            # input('Press <Enter> to continue...')
    finally:
        env.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', default='data/demos')
    parser.add_argument('--sim', action='store_true')
    parser.add_argument('--sim-showing', action='store_true')
    parser.add_argument('--show-images', action='store_true')
    parser.add_argument('--execute-obs', action='store_true')
    main(parser.parse_args())

# Author: ZMAI
# Date: 2025-03-06
# 1.1
import argparse
import os
import time
from episode_storage import EpisodeWriter
from robot_utils import run_episode_base
from is_kinova_teleop import KinovaTeleopPolicy

def main(args):
    # 创建环境
    if args.sim:
        from is_mujoco_env import KinovaMujocoEnv
        print(f"正在创建模拟环境，模型路径: models/kinova_gen3/scene_2f85.xml")
        # 检查模型文件是否存在
        if not os.path.exists('models/kinova_gen3/scene_2f85.xml'):
            print(f"警告: 模型文件不存在，请确保路径正确")
        env = KinovaMujocoEnv(show_images=args.show_images)
        
        # 等待模拟环境完全初始化
        print("等待环境初始化...")
        time.sleep(1.0)
    else:
        from is_real_env import KinovaRealEnv, HAS_KINOVA_API
        if not HAS_KINOVA_API:
            raise ImportError("无法创建真实环境：未找到Kinova API。请安装kortex_api或使用--sim参数运行模拟环境。")
        env = KinovaRealEnv()

    # 创建策略
    print("创建遥操作策略...")
    policy = KinovaTeleopPolicy()
    
    # 提前初始化策略
    print("初始化遥操作策略...")
    policy.reset()
    print("遥操作策略已初始化")

    try:
        while True:
            writer = EpisodeWriter(args.output_dir) if args.save else None
            # 使用共享模块的run_episode_base函数，传入中文参数和额外延迟
            run_episode_base(env, policy, writer, lang='zh', extra_delay=1.0)
    finally:
        print("关闭环境...")
        env.close()

if __name__ == '__main__':
    print("开始运行...1.5")
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='使用模拟环境')
    parser.add_argument('--save', action='store_true', help='保存记录')
    parser.add_argument('--show-images', action='store_true', help='显示相机图像')
    parser.add_argument('--output-dir', default='data/kinova-demos', help='记录保存目录')
    main(parser.parse_args())
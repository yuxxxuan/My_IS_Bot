#!/usr/bin/env python3
"""
测试MRoom环境的脚本
"""
import time
import argparse
from m_room import MRoom

def main():
    """
    测试MRoom环境的主函数
    """
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='测试MRoom环境')
    parser.add_argument('--use-converted-ply', action='store_true', 
                        help='使用从PLY转换而来的OBJ文件')
    parser.add_argument('--use-default-obj', action='store_true',
                        help='使用默认的poisson_mesh.obj文件')
    args = parser.parse_args()
    
    # 确定是否使用转换后的PLY文件
    use_converted_ply = not args.use_default_obj
    if args.use_converted_ply:
        use_converted_ply = True
    
    print("初始化MRoom环境...")
    env = MRoom(show_viewer=True, use_converted_ply=use_converted_ply)
    
    print("重置环境...")
    obs = env.reset()
    print(f"初始观测: {obs}")
    
    print("开始模拟...")
    try:
        for i in range(1000):
            obs = env.step()
            if i % 100 == 0:
                print(f"步骤 {i}: {obs}")
            env.render()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("用户中断模拟")
    finally:
        print("关闭环境...")
        env.close()
    
    print("测试完成")

if __name__ == "__main__":
    main() 
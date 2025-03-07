#!/usr/bin/env python3
"""
将 PLY 文件转换为 OBJ 文件，以便在 MuJoCo 中使用
需要安装 trimesh 库: pip install trimesh
"""
import os
import trimesh
import numpy as np

def convert_ply_to_obj(ply_path, obj_path):
    """
    将 PLY 文件转换为 OBJ 文件
    
    参数:
        ply_path: PLY 文件路径
        obj_path: 输出的 OBJ 文件路径
    """
    print(f"正在加载 PLY 文件: {ply_path}")
    try:
        # 加载 PLY 文件
        mesh = trimesh.load(ply_path)
        
        # 保存为 OBJ 文件
        print(f"正在将网格保存为 OBJ 文件: {obj_path}")
        mesh.export(obj_path)
        
        print(f"转换完成! OBJ 文件已保存到: {obj_path}")
        return True
    except Exception as e:
        print(f"转换过程中出错: {e}")
        return False

def main():
    # 获取当前文件所在目录的路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 构建文件路径
    ply_path = os.path.join(current_dir, "room.ply")
    obj_path = os.path.join(current_dir, "room.obj")
    
    # 转换文件
    success = convert_ply_to_obj(ply_path, obj_path)
    
    if success:
        print("现在您可以在 m_room.py 中使用 room.obj 文件了")
        print("请将 m_room.py 中的 'poisson_mesh.obj' 更改为 'room.obj'")

if __name__ == "__main__":
    main() 
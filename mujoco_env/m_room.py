import os
import numpy as np
import mujoco
import mujoco.viewer
import time

class MRoom:
    """
    静态的Mujoco仿真环境，用于加载和显示从room.ply转换而来的3D模型
    
    注意：MuJoCo不直接支持PLY格式，需要先将PLY文件转换为OBJ格式
    可以使用 convert_ply_to_obj.py 脚本进行转换
    """
    def __init__(self, show_viewer=True, use_converted_ply=True):
        """
        初始化MRoom环境
        
        参数:
            show_viewer: 是否显示Mujoco查看器
            use_converted_ply: 是否使用从PLY转换而来的OBJ文件，如果为False则使用默认的poisson_mesh.obj
        """
        self.show_viewer = show_viewer
        
        # 获取当前文件所在目录的路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 构建mesh文件的路径
        if use_converted_ply and os.path.exists(os.path.join(current_dir, "room.obj")):
            # 使用从PLY转换而来的OBJ文件
            mesh_path = os.path.join(current_dir, "room.obj")
            print("使用从room.ply转换而来的room.obj文件")
        else:
            # 使用默认的OBJ文件
            mesh_path = os.path.join(current_dir, "poisson_mesh.obj")
            if use_converted_ply:
                print("警告：room.obj文件不存在，使用默认的poisson_mesh.obj文件")
                print("请先运行 convert_ply_to_obj.py 脚本将room.ply转换为room.obj")
            else:
                print("使用默认的poisson_mesh.obj文件")
        
        # 创建一个简单的XML模型，导入mesh
        self.xml = f"""
        <mujoco>
            <asset>
                <mesh name="room_mesh" file="{mesh_path}" scale="1 1 1"/>
                <texture name="grid" type="2d" builtin="checker" rgb1=".1 .2 .3" 
                         rgb2=".2 .3 .4" width="300" height="300"/>
                <material name="grid" texture="grid" texrepeat="1 1"/>
            </asset>
            
            <worldbody>
                <light pos="0 0 3" dir="0 0 -1" diffuse="0.5 0.5 0.5"/>
                <light pos="0 0 3" dir="0 0 -1" diffuse="0.5 0.5 0.5"/>
                <geom type="plane" size="5 5 0.1" pos="0 0 0" material="grid"/>
                
                <!-- 导入mesh作为静态物体 -->
                <body pos="0 0 0" quat="1 0 0 0">
                    <geom type="mesh" mesh="room_mesh" rgba="0.8 0.8 0.8 1"/>
                </body>
                
                <!-- 添加一个小球作为参考物体 -->
                <body pos="0 0 1">
                    <joint type="free"/>
                    <geom type="sphere" size="0.1" rgba="1 0 0 1"/>
                </body>
            </worldbody>
        </mujoco>
        """
        
        # 从XML字符串加载模型
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        
        # 初始化查看器
        self.viewer = None
        if self.show_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
    
    def reset(self):
        """重置环境"""
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs()
    
    def _get_obs(self):
        """获取观测"""
        return {
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
        }
    
    def step(self):
        """执行一步模拟"""
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()
        return self._get_obs()
    
    def render(self):
        """渲染当前帧"""
        if self.viewer is not None:
            self.viewer.sync()
    
    def close(self):
        """关闭环境和查看器"""
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

def main():
    """测试MRoom环境"""
    env = MRoom(show_viewer=True)
    env.reset()
    
    try:
        # 运行一段时间的模拟
        for _ in range(1000):
            env.step()
            env.render()
            time.sleep(0.01)
    finally:
        env.close()

if __name__ == "__main__":
    main() 
"""
可视化 teleop连接成功后的陀螺仪姿态
"""
import mujoco
import mujoco.viewer
import numpy as np
from policies import TeleopPolicy

policy = TeleopPolicy()
policy.reset()  # Wait for user to press "Start episode"

# 定义一个mujoco场景
xml = """
<mujoco>
  <asset>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texrepeat="5 5"/>
  </asset>
  <worldbody>
    <light directional="true"/>
    <geom name="floor" size="0 0 .05" type="plane" material="groundplane"/> 
    <body name="target" pos="0 0 .5" mocap="true">
      <geom type="box" size=".05 .05 .05" rgba=".6 .3 .3 .5"/>
    </body>
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(xml)  # 从XML字符串创建模型
d = mujoco.MjData(m)                     # 创建对应的仿真数据
mocap_id = m.body('target').mocapid[0]   # 获取目标体的mocap ID

with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
    while viewer.is_running():
        mujoco.mj_step(m, d)
        obs = {
            'base_pose': np.zeros(3),
            'arm_pos': d.mocap_pos[mocap_id],
            'arm_quat': d.mocap_quat[mocap_id][[1, 2, 3, 0]],  # (w, x, y, z) -> (x, y, z, w)
            'gripper_pos': np.zeros(1),
        }
        action = policy.step(obs)
        if action == 'reset_env':
            break
        if isinstance(action, dict):
            d.mocap_pos[mocap_id] = action['arm_pos']
            d.mocap_quat[mocap_id] = action['arm_quat'][[3, 0, 1, 2]]  # (x, y, z, w) -> (w, x, y, z)
        viewer.sync()
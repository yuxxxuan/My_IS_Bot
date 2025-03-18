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
    # 启动 MuJoCo 被动查看器，m 是模型，d 是数据。
    # show_left_ui 和 show_right_ui 设置为 False，表示不显示左侧和右侧的用户界面。

    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_BODY
    # 设置查看器的帧选项为 mjFRAME_BODY，表示相机视角将围绕物体的主体进行。

    while viewer.is_running():
        # 当查看器仍在运行时，持续执行以下代码块。

        mujoco.mj_step(m, d)
        # 执行一个仿真步骤，更新模型 m 和数据 d 的状态。

        obs = {
            'base_pose': np.zeros(3),  # 创建一个三维零向量，表示基座位置（未使用实际值）。
            'arm_pos': d.mocap_pos[mocap_id],  # 获取运动捕捉 ID 对应的手臂位置。
            'arm_quat': d.mocap_quat[mocap_id][[1, 2, 3, 0]],  # 获取运动捕捉 ID 对应的手臂四元数，并调整顺序为 (x, y, z, w)。
            'gripper_pos': np.zeros(1),  # 创建一个零向量，表示夹爪位置（未使用实际值）。
        }

        action = policy.step(obs)
        # 使用策略（policy）根据当前观测（obs）计算出一个动作（action）。

        if action == 'reset_env':
            # 如果计算出的动作是重置环境的指令，则退出循环。
            break

        if isinstance(action, dict):
            # 如果动作是一个字典，表示包含具体的控制指令。
            d.mocap_pos[mocap_id] = action['arm_pos']  # 更新运动捕捉 ID 对应的手臂位置。
            d.mocap_quat[mocap_id] = action['arm_quat'][[3, 0, 1, 2]]  # 更新运动捕捉 ID 对应的手臂四元数，并调整顺序为 (w, x, y, z)。

        viewer.sync()
        # 同步查看器，确保显示更新后的状态。
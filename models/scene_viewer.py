import mujoco
import mujoco.viewer

# 创建一个空的模型
model = mujoco.MjModel()  # 创建一个空模型
data = mujoco.MjData(model)  # 创建对应的仿真数据

# 启动查看器
with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)  # 运行仿真步骤
import mujoco
import mediapy as media
import matplotlib.pyplot as plt

# 定义 MuJoCo 模型的 XML 描述字符串
xml = """
<mujoco>
  <worldbody>
    <!-- 定义一个光源，名称为 "top"，位于 (0, 0, 1) -->
    <light name="top" pos="0 0 1"/>
    <!-- 定义一个名为 "box_and_sphere" 的物体，该物体整体旋转 -30 度（绕 Z 轴，使用欧拉角） -->
    <body name="box_and_sphere" euler="0 0 -30">
      <!-- 在物体上定义一个铰链关节，名称为 "swing"，
           关节类型为 hinge（铰链型），旋转轴为 (1, -1, 0)；
           同时关节在物体内的局部位置为 (-0.2, -0.2, -0.2) -->
      <joint name="swing" type="hinge" axis="1 -1 0" pos="-.2 -.2 -.2"/>
      <!-- 定义一个红色盒子，名称为 "red_box"，类型为盒子，
           尺寸为 (0.2, 0.2, 0.2)（通常代表半边长），颜色为纯红（RGBA: 1 0 0 1） -->
      <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
      <!-- 定义一个绿色球体，名称为 "green_sphere"，
           在物体上的局部位置为 (0.2, 0.2, 0.2)，尺寸（半径）为 0.1，
           颜色为纯绿（RGBA: 0 1 0 1）。未显式设置类型，默认解释为球体。 -->
      <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
    </body>
  </worldbody>
</mujoco>
"""

# 从 XML 字符串中创建 MuJoCo 模型对象，该对象包含静态结构和参数信息
model = mujoco.MjModel.from_xml_string(xml)

# 创建一个 mjData 对象，用于存储和管理仿真状态（如时间、位置、速度等）
data = mujoco.MjData(model)

# 开启关节可视化选项
# 创建一个场景配置选项对象
scene_option = mujoco.MjvOption()
# 将显示关节的标志打开，这样在渲染时会可视化关节
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

# 定义仿真持续时间（秒）和帧率（Hz）
duration = 3.8  # 仿真运行 3.8 秒
framerate = 60  # 每秒渲染 60 帧

# 用于存储渲染后的每一帧图像
frames = []

# 重置仿真数据，确保仿真从初始状态开始
mujoco.mj_resetData(model, data)

# 创建一个渲染器，利用上下文管理器确保资源正确释放
with mujoco.Renderer(model) as renderer:
  # 进入仿真循环，直到仿真时间达到预设的持续时间
  while data.time < duration:
    # 执行仿真一步，更新 data 中的状态（时间、位置、速度等）
    mujoco.mj_step(model, data)
    
    # 根据当前仿真时间和目标帧率判断是否需要捕捉当前帧
    if len(frames) < data.time * framerate:
      # 更新渲染场景，传入最新的仿真数据和场景配置选项
      renderer.update_scene(data, scene_option=scene_option)
      # 渲染当前场景，得到当前帧的像素图
      pixels = renderer.render()
      # 将渲染的帧存入列表
      frames.append(pixels)

# 使用 media.show_video 函数显示捕获到的视频帧，设置帧率为预设的 framerate
media.show_video(frames, fps=framerate)

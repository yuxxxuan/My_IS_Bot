### 变量简称解释

otg - online trajectory generation input

otg_input - online trajectory generation input

otg_output - online trajectory generation output

otg_res - online trajectory generation result

model.ncam - number of cameras


### 链路分析

MujocoEnv ->  physics_loop, start MujocoSim ->  start BaseController & ArmController & shm_state

### 冷知识

1. 在 MuJoCo 中，site 是一个用于表示特定位置的对象。它可以是一个点、一个坐标系或一个参考位置，通常用于与其他物体或关节进行交互。

    用途: site 可以用于多种目的，例如：
    - 定义机器人手臂的末端执行器位置。
    - 作为传感器的安装位置。
    - 用于计算物体之间的距离或相对位置。
    - 在仿真中进行可视化和调试。

2. render_loop 和 physics_loop 是两个关键的循环，它们分别负责渲染图像和处理物理仿真。 physics_loop 和 render_loop 是并行执行的！

    - physics_loop 负责创建和启动 MuJoCo 仿真，初始化仿真环境，并在需要时启动渲染循环
  
      - 在调用 sim.launch() 后，仿真开始执行物理计算，包括重力、碰撞、关节力等

      - 该循环还会处理来自命令队列的命令，确保仿真能够根据外部输入进行调整。

    - render_loop 负责持续渲染相机图像。它会创建渲染器并在一个无限循环中调用它们的 render 方法

      - 在调用 renderer.launch() 后，渲染器开始渲染相机图像，并将其显示在屏幕上。

      - 渲染的频率取决于 render_time，如果渲染时间超过 0.1 秒（即 10 FPS），则会发出警告，提示渲染可能过慢。
      - 时间差: 由于这两个循环是并行的，渲染可能会与物理仿真有时间差。具体来说，渲染循环可能会在物理仿真更新之间进行多次渲染，这可能导致渲染的图像并不总是反映最新的物理状态。
      - 影响: 如果渲染速度较慢，可能会导致渲染的图像滞后于实际的物理状态，从而影响用户对仿真情况的理解和控制。


### WebServer

- button 的逻辑链路
policies.py
  render_template('index.html')
|
templates/index.html
  import { WebXRButton } from '/static/js/webxr-button.js';
|
static/js/webxr-button.js

```
WebXRButton 是在 static/js/webxr-button.js 文件中定义的一个类，用于创建和管理 WebXR 入口按钮。
它通过 DOM 操作将按钮插入到网页的特定位置（如 <header> 中），并通过 CSS 控制其外观和位置。
```

```
    <body name="Table" pos="0.475 0 0.2">
        <geom size="0.9 0.4 0.015" type="box" condim="4" friction="0.25 0.005 0.0001" solref="0.01 1" solimp="0.95 0.95 0.01 0.5 2" rgba="0.8 0.6 0.2 1"/>
        <geom size="0.0135 0.2" pos="-0.875 -0.375 -0.2" type="cylinder" rgba="0.8 0.6 0.2 1"/>
        <geom size="0.0135 0.2" pos="-0.875 0.375 -0.2" type="cylinder" rgba="0.8 0.6 0.2 1"/>
        <geom size="0.0135 0.2" pos="0.875 -0.375 -0.2" type="cylinder" rgba="0.8 0.6 0.2 1"/>
        <geom size="0.0135 0.2" pos="0.875 0.375 -0.2" type="cylinder" rgba="0.8 0.6 0.2 1"/>
    </body>
```


### 错综复杂的policies.py

TeleopPolicy.init(): threading.Thread(target=self.listener_loop, daemon=True).start() 

TeleopPolicy.listener_loop(): 实时监听，给到 TeleopPolicy._process_message()

TeleopPolicy._process_message(): tmd 又给到了self.teleop_controller.process_message(data) [teleop_controller 是TeleopController()类]

TeleopController.process_message(): 


### 错误
start 之后，手机没有动，就立刻开始移动了？

```
The robot arm moves without corresponding phone movement.
机器人手臂移动，而手机没有相应的动作。

The arm goes to a strange, near-zero joint configuration (upright, possibly).
手臂移动到一个奇怪、几乎为零的关节配置（可能是直立）。

Gripper control seems to work.
夹爪控制似乎正常工作。

The obs printed in the terminal are identical for several steps before the robot starts to move on its own. This is VERY important.
终端中打印的 obs 在机器人开始自主移动之前保持几步相同。这非常重要。
```

### 一个完整的初始化错误？

```
Press "Start episode" in the web app when ready to start new episode
[TeleopPolicy-reset] Finished
Starting new episode
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02484991  0.03699074 -0.00376394], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02484991  0.03699074 -0.00376394], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02478235  0.03700768 -0.00403322], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02473748  0.03702245 -0.00417075], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02469853  0.03702757 -0.00435219], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[MujocoEnv-get_obs] arm_pos [0.12077208 0.00135448 1.12737197] arm_quat [0.70667739 0.70677465 0.02318187 0.02322175]
[main-run_episode]: env.get_obs() Get!
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02469718  0.03701542 -0.00446185], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[ArmController-control_callback] [IK] 目标位置: [0.12092346 0.00139131 1.12678372], 解算结果: [ 0.71022199 -0.35761782  2.0071718  -0.23722379  1.36863227 -2.36864407
  2.52488457]
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02474285  0.03698003 -0.00450205], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02479447  0.0369518  -0.00444955], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02483297  0.03693437 -0.00437901], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02484727  0.03693289 -0.00430988], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02482645  0.03695114 -0.00427322], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[MujocoEnv-get_obs] arm_pos [1.21897010e-01 2.38139312e-04 1.12940577e+00] arm_quat [0.70170829 0.71172222 0.01911476 0.02629804]
[main-run_episode]: env.get_obs() Get!
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02481473  0.03695824 -0.00427992], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[ArmController-control_callback] [IK] 目标位置: [0.12079554 0.00131488 1.12686269], 解算结果: [ 0.66922402 -0.34371076  2.026192   -0.23094443  1.40679689 -2.34829277
  2.55603433]
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02480257  0.03696492 -0.00429268], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.0247937   0.03696771 -0.00431982], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02477412  0.03697205 -0.00439439], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02472652  0.03699595 -0.00446088], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02466962  0.03703164 -0.00447963], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[MujocoEnv-get_obs] arm_pos [ 0.12852266 -0.0040123   1.14055382] arm_quat [ 0.67668511  0.73504931 -0.00152077  0.04239659]
[main-run_episode]: env.get_obs() Get!
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02466962  0.03703164 -0.00447963], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[ArmController-control_callback] [IK] 目标位置: [0.12095236 0.00139538 1.12665628], 解算结果: [ 1.50348136 -0.12017003  1.50324367 -0.21229221  0.82079052 -2.42773682
  2.38634448]
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02465698  0.0370405  -0.00447598], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02463725  0.03705322 -0.00447943], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02461577  0.03706776 -0.00447716], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02277147  0.03827267 -0.00408201], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02287098  0.03820597 -0.00390998], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[MujocoEnv-get_obs] arm_pos [ 0.14454997 -0.00689481  1.16233258] arm_quat [ 0.6432465   0.76215552 -0.02516988  0.06869771]
[main-run_episode]: env.get_obs() Get!
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02296836  0.03814225 -0.00374828], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[ArmController-control_callback] [IK] 目标位置: [0.12275101 0.00256971 1.12722593], 解算结果: [-2.80403442  0.00914098  5.01277218 -0.04233249  1.18927792 -2.82419361
  1.88556262]
[TeleopController-process_message] 手机WebXR输入 pos: [-0.02306227  0.03808422 -0.00356473], rot: <scipy.spatial.transform._rotation.Rotation object at 0x7e0945984210>
[MujocoEnv-get_obs] arm_pos [ 0.16695383 -0.00712814  1.19209042] arm_quat [ 0.61407763  0.78173353 -0.04486252  0.09893789]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.18399524 -0.00729082  1.21884002] arm_quat [ 0.58906367  0.79655442 -0.06382429  0.12013122]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.19861196 -0.00794453  1.24600223] arm_quat [ 0.56751157  0.80811016 -0.07749588  0.13741534]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.20805533 -0.00774007  1.2670831 ] arm_quat [ 0.5590329   0.81112205 -0.07875698  0.15284167]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.2118649  -0.00785799  1.27655253] arm_quat [ 0.55686442  0.81122717 -0.0780069   0.16039771]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21183742 -0.00819842  1.27782768] arm_quat [ 0.55618105  0.81150388 -0.07975151  0.16051102]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21160405 -0.00836915  1.2779692 ] arm_quat [ 0.55592824  0.81165569 -0.08074101  0.16012408]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21149505 -0.00843455  1.27798553] arm_quat [ 0.5558358   0.81171599 -0.08113145  0.15994187]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.2114532  -0.00845836  1.27798786] arm_quat [ 0.55580197  0.81173851 -0.08127504  0.1598722 ]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.2114379  -0.00846692  1.27798833] arm_quat [ 0.55578961  0.81174679 -0.08132688  0.15984678]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21143233 -0.00847002  1.27798846] arm_quat [ 0.55578503  0.81174986 -0.08134571  0.15983752]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21143041 -0.00847108  1.2779885 ] arm_quat [ 0.55578343  0.81175094 -0.08135217  0.15983434]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.2114297  -0.00847148  1.27798852] arm_quat [ 0.55578283  0.81175134 -0.08135457  0.15983316]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142945 -0.00847162  1.27798852] arm_quat [ 0.55578261  0.81175149 -0.08135543  0.15983273]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142936 -0.00847167  1.27798852] arm_quat [ 0.55578253  0.81175154 -0.08135573  0.15983259]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142932 -0.00847168  1.27798852] arm_quat [ 0.5557825   0.81175156 -0.08135584  0.15983253]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578249  0.81175157 -0.08135588  0.15983251]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578249  0.81175157 -0.0813559   0.1598325 ]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578249  0.81175157 -0.0813559   0.1598325 ]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578248  0.81175157 -0.0813559   0.1598325 ]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578248  0.81175157 -0.0813559   0.1598325 ]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578248  0.81175157 -0.0813559   0.1598325 ]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578248  0.81175157 -0.0813559   0.1598325 ]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578248  0.81175157 -0.0813559   0.1598325 ]
[main-run_episode]: env.get_obs() Get!
[MujocoEnv-get_obs] arm_pos [ 0.21142931 -0.00847169  1.27798852] arm_quat [ 0.55578248  0.81175157 -0.0813559   0.1598325 ]
[main-run_episode]: env.get_obs() Get!
Episode ended

```

[MujocoEnv-get_obs] arm_pos [ 0.20303693 -0.01083612  1.20448979] arm_quat [-0.52992813 -0.53133276  0.46624263  0.46848637]

[MujocoEnv-get_obs] arm_pos [0.12750192 0.00404312 1.20320346] arm_quat [0.0127183  0.01295778 0.7074794  0.7065007 ]


地面的：
[MujocoEnv-get_obs] arm_pos [ 0.1375593  -0.00703275  0.22115066] arm_quat [0.09867632 0.09243806 0.68849486 0.7125258 ]

table:
[MujocoEnv-get_obs] arm_pos [ 0.1375593  -0.00703275  0.22115066] arm_quat [0.09867632 0.09243806 0.68849486 0.7125258 ]

[MujocoEnv-get_obs] arm_pos [0.01162846 0.01026551 1.78043806] arm_quat [-0.08776893  0.25911747  0.70696023  0.65219781]


2. [MujocoEnv-get_obs] arm_pos [0.18100368 0.21177663 1.61031325] arm_quat [-0.80375937 -0.21944521  0.21680645  0.50873337]

[TeleopController-process_message] 手机WebXR输入 pos: [2.79588331e-02 3.45646255e-02 9.91384746e-05], rot: <scipy.spatial.transform._rotation.Rotation object at 0x71dd2da44030>

### 2025-03-19

要求的是 pip install protobuf==3.20.0

这尼玛python>3.10 升级的 protobuf== 6.30.1

会不会是一个隐患？

```
        # 获取材质ID
        floor_mat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, self.floor_material_name)
        print(f"currentfloor_material_name: {self.floor_material_name}")
        table_mat_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, self.table_material_name)
        print(f"current table_material_name: {self.table_material_name}")

        # 随机选择纹理名称
        selected_floor_texture = random.choice(self.floor_textures)
        selected_table_texture = random.choice(self.table_textures)
        print(f"selected_floor_texture: {selected_floor_texture}, selected_table_texture: {selected_table_texture}")

        # 获取纹理ID
        floor_tex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, selected_floor_texture)
        table_tex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TEXTURE, selected_table_texture)
        print(f"floor_tex_id: {floor_tex_id}, table_tex_id: {table_tex_id}")
        print(f"floor_mat_id: {floor_mat_id}, table_mat_id: {table_mat_id}")
        
        # 设置材质的纹理ID
        # 检查纹理ID是否有效
        if 0 <= floor_tex_id < model.ntex:
            model.mat[floor_mat_id].texid = floor_tex_id
        else:
            print(f"Error: Invalid floor_tex_id: {floor_tex_id}, skipping material update.")

        if 0 <= table_tex_id < model.ntex:
            model.mat[table_mat_id].texid = table_tex_id
        else:
            print(f"Error: Invalid table_tex_id: {table_tex_id}, skipping material update.")

```
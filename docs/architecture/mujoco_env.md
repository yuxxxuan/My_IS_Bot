# MuJoCo模拟环境

## 概述

`mujoco_env.py`是IS_Bot项目的核心组件之一，负责创建和管理基于MuJoCo物理引擎的仿真环境。该模块实现了物理世界的初始化、机器人控制接口、环境状态观测等核心功能，为上层应用提供了统一的交互接口。

## 主要组件

### 共享内存状态类 (ShmState)

用于在不同进程间共享机器人状态信息，主要包括：

```python
class ShmState:
    def __init__(self, existing_instance=None):
        # 创建共享内存用于存储机器人状态
        self.arm_pos      # 机械臂末端位置
        self.arm_quat     # 机械臂末端姿态（四元数）
        self.gripper_pos  # 夹爪开合状态
        self.initialized  # 初始化标志
```

### 共享内存图像类 (ShmImage)

用于在不同进程间共享相机图像：

```python
class ShmImage:
    def __init__(self, camera_name=None, width=None, height=None, existing_instance=None):
        # 创建共享内存用于存储相机图像
        self.camera_name  # 相机名称
        self.data         # 图像数据
```

### 渲染器类 (Renderer)

负责渲染MuJoCo场景并将图像写入共享内存：

```python
class Renderer:
    def __init__(self, model, data, shm_image):
        # 初始化渲染器
        self.render()     # 渲染场景
```

### 手臂控制器类 (ArmController)

负责机械臂的控制和轨迹规划：

```python
class ArmController:
    def __init__(self, qpos, qvel, ctrl, qpos_gripper, ctrl_gripper, timestep):
        # 初始化控制器参数
        self.ik_solver    # 逆运动学求解器
        self.otg          # 在线轨迹生成器

    def control_callback(self, command):
        # 处理控制命令
        # 使用IK求解器计算关节位置
        # 使用轨迹生成器平滑轨迹
```

### MuJoCo仿真类 (MujocoSim)

核心仿真类，负责物理仿真的执行：

```python
class MujocoSim:
    def __init__(self, mjcf_path, command_queue, shm_state, show_viewer=True):
        # 初始化物理仿真环境
        self.model        # MuJoCo模型
        self.data         # MuJoCo数据
        
    def randomize_environment(self):
        # 随机化环境参数
        self.randomize_materials()       # 随机更新材质
        self.randomize_objects_textures() # 随机更新物体纹理
        self.randomize_lighting()        # 随机更新光照
```

### MuJoCo环境类 (MujocoEnv)

对外提供的主要接口类，封装了底层实现细节：

```python
class MujocoEnv:
    def __init__(self, render_images=True, show_viewer=True, show_images=False):
        # 初始化环境
        self.mjcf_path    # MuJoCo模型路径
        
    def reset(self):
        # 重置环境状态
        
    def step(self, action):
        # 执行动作
        
    def get_obs(self):
        # 获取观测数据
```

## 环境随机化

环境随机化是增强模型泛化能力的重要手段，`mujoco_env.py`实现了三种随机化方式：

1. **材质随机化**：随机更改场景中物体的材质，如地板和桌面
2. **物体纹理随机化**：随机更改特定物体的颜色和纹理
3. **光照随机化**：随机调整光源的位置、强度和颜色

```python
def randomize_materials(self):
    # 随机更新环境纹理
    
def randomize_objects_textures(self):
    # 随机更新指定物体纹理
    
def randomize_lighting(self):
    # 随机更新环境光源
```

## 多进程架构

为了提高性能和响应速度，`mujoco_env.py`采用了多进程架构：

1. **物理循环进程**：负责物理引擎的更新
2. **渲染循环线程**：负责场景渲染
3. **可视化循环进程**：负责图像显示（可选）

这种设计使得物理模拟与渲染可以并行执行，提高了系统的效率。

## 使用示例

```python
# 创建环境
env = MujocoEnv(render_images=True, show_viewer=True)

# 重置环境
env.reset()

# 执行动作
action = {
    'arm_pos': np.array([0.5, 0, 0.3]),
    'arm_quat': np.array([0, 0, 0, 1]),
    'gripper_pos': np.array([0])
}
env.step(action)

# 获取观测
obs = env.get_obs()
print(obs['arm_pos'])  # 打印末端位置
```

## 扩展与定制

`mujoco_env.py`设计了良好的扩展接口，方便用户进行定制：

1. **更换机器人模型**：修改`mjcf_path`可以载入不同的机器人模型
2. **添加自定义随机化**：扩展随机化函数可以实现更复杂的域随机化
3. **定制观测空间**：修改`get_obs`函数可以返回自定义的观测数据
4. **定制奖励函数**：可以基于环境状态实现自定义的奖励函数


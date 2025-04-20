# 核心模块

本节介绍IS_Bot项目的核心模块结构，帮助您理解项目的代码组织和模块功能。

## 核心模块概览

IS_Bot项目的核心模块主要包括以下部分：

| 模块名称 | 文件 | 主要功能 |
| ------- | ---- | ------- |
| 物理仿真环境 | `mujoco_env.py` | 提供基于MuJoCo的物理仿真环境 |
| 逆运动学求解器 | `ik_solver.py` | 实现机械臂的逆运动学算法 |
| 机器人控制器 | `arm_controller.py` | 机械臂的底层控制逻辑 |
| 策略服务器 | `policy_server.py` | 提供控制策略的HTTP服务 |
| 工具函数 | `utils.py` | 通用工具函数集合 |
| 常量定义 | `constants.py` | 项目中使用的常量定义 |

## 模块依赖关系

各核心模块之间的依赖关系如下：

```
           constants.py
                 │
                 ▼
           ┌───────────┐
           │ utils.py  │
           └───────────┘
                 │
                 ▼
  ┌───────┐  ┌───────────┐  ┌───────────┐
  │ik_solver│◀─┤mujoco_env├─▶│arm_controller│
  └───────┘  └───────────┘  └───────────┘
                 │                 │
                 ▼                 ▼
           ┌───────────┐    ┌───────────┐
           │   main.py │    │policy_server│
           └───────────┘    └───────────┘
```

## 核心文件详解

### mujoco_env.py

`mujoco_env.py` 是整个项目的核心组件，负责创建和管理基于MuJoCo的物理仿真环境。

**主要类：**
- `ShmState`: 共享内存状态类
- `ShmImage`: 共享内存图像类
- `Renderer`: 渲染器类
- `ArmController`: 手臂控制器类
- `MujocoSim`: MuJoCo仿真类
- `MujocoEnv`: MuJoCo环境类（主接口）

更多详细信息请参阅[MuJoCo模拟环境](../architecture/mujoco_env.md)文档。

### ik_solver.py

`ik_solver.py` 实现了机械臂的逆运动学算法，将末端执行器的目标位置和姿态转换为关节角度。

**主要类：**
- `IKSolver`: 逆运动学求解器

```python
class IKSolver:
    def __init__(self, ee_offset=0.0):
        # 初始化逆运动学求解器
        
    def solve(self, target_pos, target_quat, current_joints):
        # 计算满足目标位置和姿态的关节角度
```

### arm_controller.py

`arm_controller.py` 提供了机械臂的底层控制接口，负责与实际的机器人硬件通信。

**主要类：**
- `ArmController`: 机械臂控制器

```python
class ArmController:
    def __init__(self, ip='192.168.1.10', port=10000):
        # 初始化机械臂控制器
        
    def connect(self):
        # 连接到机械臂
        
    def move_to_pose(self, position, orientation):
        # 控制机械臂移动到指定位姿
```

### policy_server.py

`policy_server.py` 实现了一个HTTP服务器，用于提供控制策略的接口。

**主要类：**
- `PolicyServer`: 策略服务器

```python
class PolicyServer:
    def __init__(self, policy, host='0.0.0.0', port=8000):
        # 初始化策略服务器
        
    def start(self):
        # 启动服务器
        
    def handle_request(self, request):
        # 处理控制请求
```

### constants.py

`constants.py` 定义了项目中使用的各种常量，如控制频率、默认参数等。

```python
# 控制频率相关
POLICY_CONTROL_PERIOD = 0.1  # 策略控制周期 (10Hz)

# 机械臂参数
ARM_DOF = 7  # 机械臂自由度

# 默认路径
DEFAULT_MODEL_PATH = 'models/kinova_gen3/1_table_1_gen3.xml'
```

### utils.py

`utils.py` 提供了各种通用工具函数，用于数据处理、坐标变换等。

```python
def rpy_to_quat(rpy):
    """欧拉角转四元数"""
    
def quat_to_rpy(quat):
    """四元数转欧拉角"""
    
def transform_pose(pose, transform):
    """坐标变换"""
```

## 扩展与自定义

IS_Bot的核心模块设计遵循模块化原则，您可以通过以下方式进行扩展：

1. **添加新的环境模型**：创建新的XML模型文件并在`mujoco_env.py`中引用
2. **实现自定义控制器**：继承`ArmController`类并重写相关方法
3. **添加新的策略**：在`policies.py`中实现新的策略类 
# 项目结构总览

IS_Bot项目采用了模块化的文件组织结构，便于开发者理解和扩展。本页面将介绍项目的整体文件结构及各个部分的作用。

## 目录结构

```
IS_Bot/
├── models/               # MuJoCo模型文件
│   ├── kinova_gen3/     # Kinova Gen3机器人模型
│   └── textures/        # 纹理文件
├── docs/                # 项目文档
├── test/                # 测试代码
├── assets/              # 静态资源
├── .github/             # GitHub相关配置
└── [核心Python文件]      # 项目主要代码文件
```

## 模型文件

`models/` 目录包含了所有的MuJoCo模型文件（XML格式）和相关资源：

```
models/
├── kinova_gen3/           # Kinova Gen3机器人模型
│   ├── 1_table_1_gen3.xml # 主要场景文件
│   ├── gen3_2f85.xml      # 机器人本体定义
│   ├── fridge.xml         # 冰箱模型定义
│   └── objects/           # 场景物体
│       └── basic/         # 基础物体(苹果、香蕉等)
├── textures/              # 纹理文件
│   ├── textures.xml       # 纹理定义
│   └── [纹理图片文件]
└── assets/                # 3D模型资源
```

### 场景文件说明

- **1_table_1_gen3.xml**: 主要场景文件，定义了机器人和桌面环境
- **gen3_2f85.xml**: Kinova Gen3机器人的定义，包含机械结构和控制参数
- **fridge.xml**: 冰箱模型定义
- **objects/basic/**: 包含基础物体如苹果、香蕉、橙子等的模型和纹理

## 核心Python文件

项目根目录下的Python文件是系统的核心组件：

| 文件名 | 描述 |
|-------|------|
| `mujoco_env.py` | MuJoCo环境的主要实现，包含物理仿真逻辑 |
| `ik_solver.py` | 逆运动学求解器，用于计算机械臂的关节位置 |
| `arm_controller.py` | 机械臂控制器，提供机械臂控制接口 |
| `arm_server.py` | 机械臂控制服务器，提供HTTP API |
| `policy_server.py` | 策略服务器，用于运行控制策略 |
| `policies.py` | 控制策略的实现 |
| `main.py` | 主程序入口 |
| `constants.py` | 常量定义 |
| `utils.py` | 工具函数 |
| `cameras.py` | 相机相关功能 |
| `kinova.py` | Kinova机器人特定的代码 |
| `requirements.txt` | 项目依赖列表 |

## 测试和示例

`test/` 目录包含了各类测试代码，用于验证系统功能：

```
test/
├── test_ik_solver.py     # 测试逆运动学求解器
├── test_arm_control.py   # 测试机械臂控制
└── [其他测试文件]
```

## 数据处理

项目包含了一些数据处理和回放相关的文件：

| 文件名 | 描述 |
|-------|------|
| `episode_storage.py` | 用于存储和管理回放数据 |
| `replay_episodes.py` | 回放数据的实现 |
| `convert_to_robomimic_hdf5.py` | 将数据转换为robomimic格式 |

## 文档

`docs/` 目录包含了项目的文档文件：

```
docs/
├── index.md              # 文档首页
├── overview.md           # 项目概述
├── architecture/         # 技术架构文档
├── file_structure/       # 文件结构文档
├── tutorials/            # 使用教程
└── api/                  # API参考
```

## 配置文件

项目包含了一些配置文件：

| 文件名 | 描述 |
|-------|------|
| `.gitignore` | Git忽略文件列表 |
| `mkdocs.yml` | MkDocs文档配置 |
| `.github/workflows/deploy-docs.yml` | GitHub Actions工作流，用于部署文档 |

## 依赖管理

项目使用`requirements.txt`管理Python依赖，主要依赖包括：

- `mujoco`: MuJoCo物理引擎Python绑定
- `numpy`: 科学计算库
- `opencv-python`: 计算机视觉库
- `ruckig`: 在线轨迹生成库
- `scipy`: 科学计算库
- `flask`: Web服务器库 
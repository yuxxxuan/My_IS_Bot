# 项目结构总览

IS_Bot项目采用了模块化的文件组织结构，便于开发者理解和扩展。本页面将介绍项目的整体文件结构及各个部分的作用。

## 目录结构

```
IS_Bot/
├── models/               # MuJoCo模型文件
│   ├── assets/           # 3D模型资源
│   │   ├── gen3/         # Kinova Gen3机器人模型资源
│   │   ├── 2f85/         # 夹爪模型资源
│   │   ├── objects/      # 场景物体模型
│   │   └── base/         # 基础模型资源
│   ├── textures/         # 纹理文件
│   ├── robot_urdf/       # 机器人URDF文件
│   ├── 1_table_1_gen3.xml # 主要场景文件
│   ├── gen3_2f85.xml     # 机器人模型定义
│   ├── gen3.xml          # 不含夹爪的机器人定义
│   └── fridge.xml        # 冰箱模型定义
├── docs_src/             # 文档源文件（Markdown格式）
├── docs/                 # 生成的静态文档网站
├── assets/               # 项目静态资源
├── static/               # 静态Web资源
├── templates/            # Web模板
├── data/                 # 数据存储目录
└── [核心Python文件]      # 项目主要代码文件
```

## 模型文件

`models/` 目录包含了所有的MuJoCo模型文件（XML格式）和相关资源：

### 主要模型文件

- **1_table_1_gen3.xml**: 主要场景文件，定义了机器人和桌面环境
- **gen3_2f85.xml**: Kinova Gen3机器人的定义，包含机械结构和夹爪
- **gen3.xml**: Kinova Gen3机器人定义（不含夹爪）
- **fridge.xml**: 冰箱模型定义
- **scene_2f85.xml**: 场景设置文件

### 模型资源

`models/assets/` 包含了各种3D模型资源：
- **gen3/**: Kinova Gen3机器人的模型组件
- **2f85/**: 夹爪模型组件
- **objects/basic/**: 基础物体模型
- **objects/modern_fridge/**: 冰箱模型组件

### 纹理资源

`models/textures/` 包含了各种纹理文件和定义：
- **textures.xml**: 纹理定义文件
- 各种材质纹理图片（wood-tiles.png, clay.png等）

## 核心Python文件

项目根目录下的Python文件是系统的核心组件：

| 文件名 | 描述 |
|-------|------|
| `mujoco_env.py` | MuJoCo环境的主要实现，包含物理仿真逻辑 |
| `real_env.py` | 真实环境抽象及接口实现 |
| `ik_solver.py` | 逆运动学求解器，用于计算机械臂的关节位置 |
| `arm_controller.py` | 机械臂控制器，提供机械臂控制接口 |
| `arm_server.py` | 机械臂控制服务器，提供HTTP API |
| `base_controller.py` | 底盘控制器实现 |
| `base_server.py` | 底盘服务器实现 |
| `policy_server.py` | 策略服务器，用于运行控制策略 |
| `policies.py` | 控制策略的实现 |
| `kinova.py` | Kinova机器人特定的代码 |
| `gamepad_teleop.py` | 游戏手柄遥操作实现 |
| `cameras.py` | 相机相关功能 |
| `constants.py` | 常量定义 |
| `utils.py` | 工具函数 |
| `main.py` | 主程序入口 |

## 数据处理和实验工具

项目包含了数据处理、分析和实验相关的文件：

| 文件名 | 描述 |
|-------|------|
| `episode_storage.py` | 用于存储和管理回放数据 |
| `replay_episodes.py` | 回放数据的实现 |
| `convert_to_robomimic_hdf5.py` | 将数据转换为robomimic格式 |
| `plot_base_state.py` | 底盘状态分析绘图工具 |
| `plot_demos.ipynb` | 示范数据分析Jupyter笔记本 |
| `teleop_test.py` | 遥操作测试脚本 |
| `arm_test.py`, `arm_test_retract.py` | 机械臂测试脚本 |

## 文档

项目使用MkDocs生成文档网站：

- **docs_src/**: 文档源文件（Markdown格式）
- **docs/**: 生成的静态文档网站
- **mkdocs.yml**: MkDocs配置文件

文档结构包括：
```
docs_src/
├── index.md              # 文档首页
├── overview.md           # 项目概述
├── architecture/         # 技术架构文档
├── file_structure/       # 文件结构文档
├── tutorials/            # 使用教程
├── api/                  # API参考
└── contributing.md       # 贡献指南
```

## 手机遥操作Web接口资源

项目包含Web接口相关的资源文件：

- **static/**: 静态资源（CSS、JavaScript等）
- **templates/**: Web模板文件

## 配置文件

项目包含了一些配置文件：

| 文件名 | 描述 |
|-------|------|
| `.gitignore` | Git忽略文件列表 |
| `mkdocs.yml` | MkDocs文档配置 |
| `requirements.txt` | Python依赖列表 |
| `diffusion-policy.patch` | 扩散策略模型的补丁文件 |

## 依赖管理

项目使用`requirements.txt`管理Python依赖，主要依赖包括：

- `numpy`: 科学计算库
- `scipy`: 科学计算库 
- `opencv-python`: 计算机视觉库
- `h5py`: HDF5文件格式支持
- `matplotlib`: 数据可视化库
- `tqdm`: 进度条工具
- `notebook`: Jupyter笔记本
- `flask`: Web服务器库（用于HTTP API）

## 笔记文件

项目包含一些开发笔记和文档：

- `coding_notes.md`: 编码相关笔记
- `working_notes.md`: 工作笔记
- `Execute Instructions.md`: 执行指令说明
- `README.md`: 项目简介 
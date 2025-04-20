# IS_Bot

欢迎使用 IS_Bot 项目文档，我是作者 [Fennmai](https://github.com/FennMai)。

## 项目简介
快速入门IS_Bot项目，从多个维度进行引导和思考。

IS_Bot项目是我在 [CUHKSZ-FNii](https://fnii.cuhk.edu.cn/) 学习 Embodied AI 的一个工程性较强的项目。虽然我仅在这个项目真实的编写时间仅花了3个月左右的时间（2025-02 ～2025-04），但项目融合了非常多开源论文/项目的内容和思路，代表了我对 Embodiedment 领域的理解和做的一些浅显的探索。

## 编写初衷

我在构思做这个项目考虑到了以下这几种现实情况：

1. **硬件** : 组里的机械臂是kinova gen3, 不是 Embodied 热门的 Franka 机械臂，所以很多经典项目的dataset和model均无法直接复用，这对新手入门 Embodiedment 提升了很多学习的难度，因为Embodiedment整个项目做下来，链路非常的长，你很难去评估是你的模型问题？机械臂逆运动学求解问题？网络问题？视角问题？还是各种你想不到的情况。
2. **Scaling law** : training model 绕不开这一点，我们需要大量的数据。我尝试找到了一小部分kinva gen3 机械臂的数据，并尝试使用。但因为种种原因（数据的质量，内容和格式的不统一等等），我在数据工程做了大量的工作，但最终效果并不好。
3. **Data Collection** ：经过第二点，我觉得需要有自己建立高质量数据集的能力，这样当我有新idea，能够快速的去实践，而不是每次都成为开源数据集的苦力。同时，也能够更加深入理解每篇新论文最底层的不同点。但是，怎么评估你的数据集质量是高是低？
4. **Project and update** : 这个领域目前来看，很前沿，迭代速度也很快。我每天看着arxiv上发布的成多的新论文，会感觉自己怎么什么都不会？有些时候，我fork了一个新的项目，想要去尝试复现，理解它的本质和原理，但项目本身的复杂程度和代码的耦合性，让我麻木...

所以，我想要一个面对 Sim 和 Real 环境均可使用的 Robot Arm Controllers，一个 Sim 和 Real 一致的环境，一个能够适配不同数采硬件的数据采集方案，一个减少 Sim2Real gap 的方案，一个针对实验室资源有限也能够持续学习的路线，一个简洁并能写好注释的代码仓库......

我知道，想要实现这每一条，都很难。但正是因为难，这才是做 Emobodied AI 的乐趣 :D

最后，文档和代码有错误的地方，请多多谅解！有疑问和debug均可向我提出。

如果对你有帮助，请给我的github点点follow和star！

## 项目特点

**Robot Twin、Sim2Real、Real2Sim、Real2Sim2Real、Teleoperation、Model Deployment、Domain Randomization、Data Augmentation**

## 快速开始

```bash
# 克隆仓库
git clone https://github.com/FennMai/IS_Bot.git
cd IS_Bot

# 安装依赖
pip install -r requirements.txt

```

## 使用指南/学习路线

- [项目概述](overview.md) - 了解IS_Bot项目的背景和设计理念
- [环境配置](tutorials/setup.md) - 配置开发环境
- [技术架构](architecture/system.md) - 了解系统的技术架构
- [基本操作](tutorials/basic_usage.md) - 学习基本的使用方法


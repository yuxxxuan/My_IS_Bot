# IS_Bot

Simulation and Reality Data Super Hybrid Platform
## 1. QuickStart
1. 安装包
```
pip install -r requirements.txt
```
2. 测试项目
```bash

# [Real]机器人通信和控制测试 --- arm回到retract位置
python arm_test_retract.py

# [Real]camera 测试

# [Real]遥操作测试

# [Sim]仿真环境加载渲染和运行
python mujoco_env.py

# [Sim]遥操作连接测试
python test/teleop_test.py
```

## 2.文档
### Online
详细文档与教程已发布：[IS_Bot 文档](https://fennmai.github.io/IS_Bot/)

### Offline
```
mkdocs.yml # 文档结构
docs_src # 本地文档阅读与编辑
docs # build 后发布文档
```
#### mkdocs下文档编写
1. 新建md文件，添加到 mkdocs.yml
2. 在docs_src文件夹下对应位置，编写文档内容
3. 编译与本地测试
```
# 编译
mkdocs build 
# 本地运行测试
mkdocs serve
```

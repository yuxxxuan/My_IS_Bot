# quick start
```
 python -m mujoco.viewer
```

# execute instructions

- sim env teleop play 
  快速控制仿真遥操作
```sh
python main.py --sim --teleop
```

- sim env data collection

```sh
python main.py --sim --teleop --save --output-dir data/demo_sim
```

- sim2sim replay （need to debug）

```bash
# 带上场景切换的展示特效
python replay_episodes.py --sim --sim-showing --input-dir data/demo_sim
# 1
python replay_episodes.py --sim --input-dir data/demos --execute-obs

```

- real arm retract

```bash
python arm_test_retract.py
```

- real env data collection
  
```bash
# test the arm
python arm_test_retract.py

# terminal1 start the arm
python arm_server.py

# terminal2 start main

python main.py --teleop --save --output-dir data/demo_0321
```


- real2sim replay

```bash
# 1
python replay_episodes.py --sim --input-dir data/demo_0321 
# 1
python replay_episodes.py --sim --input-dir data/demos --execute-obs

```

- real2real replay


```bash
# 1
python replay_episodes.py --input-dir data/demo_0321

# 2025-04-03
python replay_episodes.py --input-dir data/demo_sim

```
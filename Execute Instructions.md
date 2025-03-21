- real arm retract

```bash
python arm_test_retract.py
```

- sim_env data collection

```bash
python main.py --sim --teleop --save --output-dir data/demo_sim
```

- sim2real replay （need to debug）

```bash
# 1
python replay_episodes.py --sim --input-dir data/demo_sim
# 1
python replay_episodes.py --sim --input-dir data/demos --execute-obs

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
python replay_episodes.py --sim --input-dir data/demos 
# 1
python replay_episodes.py --sim --input-dir data/demos --execute-obs

```

- real2real replay


```bash
# 1
python replay_episodes.py --input-dir data/demo_0321

```
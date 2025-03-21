- real arm retract

```bash
python arm_test_retract.py
```

- sim_env data collection

```bash
python main.py --sim --teleop --save --output-dir data/demo_0321
```

- sim2real replay （need to debug）



- real env data collection
  
```bash

```


- real2sim replay

```bash
# 1
python replay_episodes.py --sim --input-dir data/demos 
# 1
python replay_episodes.py --sim --input-dir data/demos --execute-obs

```
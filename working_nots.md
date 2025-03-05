
### error

1. pip install -r requirements.txt

```
ERROR: Could not find a version that satisfies the requirement phoenix6==24.3.0 (from versions: none)
ERROR: No matching distribution found for phoenix6==24.3.0
```
![alt text](<assets/working_nots/Screenshot from 2025-02-25 16-12-21.png>)

solve: 
```
phoenix6-25.3.1-cp310-abi3-manylinux_2_35_x86_64.whl
```

> (tidybot2) jojo@jojo-System-Product-Name:~/Downloads$ pip install phoenix6-25.3.1-cp310-abi3-manylinux_2_35_x86_64.whl 
ERROR: phoenix6-25.3.1-cp310-abi3-manylinux_2_35_x86_64.whl is not a supported wheel on this platform.


GLIBC version:
> jojo@jojo-System-Product-Name:~/Downloads$ ldd --version
ldd (Ubuntu GLIBC 2.31-0ubuntu9.17) 2.31
Copyright (C) 2020 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
Written by Roland McGrath and Ulrich Drepper.

I give up the conda, using the venv

```

```

### Test

```
# show the grasp demo 
python replay_episodes.py --sim --input-dir data/sim-v1

# show the random action for gen3
python mujoco_env.py --show-images

```

- teleop test

```
python main.py --sim --teleop
```

bug?
```
QObject::moveToThread: Current thread (0x213ac90) is not the object's thread (0x225ef50).
Cannot move to target thread (0x213ac90)

QObject::moveToThread: Current thread (0x213ac90) is not the object's thread (0x225ef50).
Cannot move to target thread (0x213ac90)

```

2025-03-03

`teleop_test.py` 陀螺仪与手机的映射


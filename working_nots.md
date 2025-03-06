
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


error
```
(botenv) jojo@jojo-System-Product-Name:~/Documents/ZMAI/IS_Bot$ python3 is_main.py --sim
重置环境...
Starting server at 192.168.1.108:5000
 * Serving Flask app 'policies'
 * Debug mode: off
ERROR: Python exception raised

Press Enter to exit ...Exception in thread Thread-2 (listener_loop):
Traceback (most recent call last):
  File "/home/jojo/anaconda3/lib/python3.12/threading.py", line 1073, in _bootstrap_inner
    self.run()
  File "/home/jojo/anaconda3/lib/python3.12/threading.py", line 1010, in run
    self._target(*self._args, **self._kwargs)
  File "/home/jojo/Documents/ZMAI/IS_Bot/policies.py", line 263, in listener_loop
    self._process_message(data)
  File "/home/jojo/Documents/ZMAI/IS_Bot/policies.py", line 268, in _process_message
    self.teleop_controller.process_message(data)
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
AttributeError: 'NoneType' object has no attribute 'process_message'

```
wait! give me 5 mins , i am debug :)!!!
```
(botenv) jojo@jojo-System-Product-Name:~/Documents/ZMAI/IS_Bot$ python3 is_main.py --sim
Starting server at 192.168.1.108:5000
 * Serving Flask app 'policies'
 * Debug mode: off
Exception in thread Thread-2 (_physics_loop):
Traceback (most recent call last):
  File "/home/jojo/anaconda3/lib/python3.12/threading.py", line 1073, in _bootstrap_inner
    self.run()
  File "/home/jojo/anaconda3/lib/python3.12/threading.py", line 1010, in run
    self._target(*self._args, **self._kwargs)
  File "/home/jojo/Documents/ZMAI/botenv/lib/python3.12/site-packages/mujoco/viewer.py", line 306, in _physics_loop
    mujoco.mj_step(m, d)
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_mujoco_env.py", line 211, in control_callback
    mujoco.mju_mat2Quat(self.site_quat, self.site_xmat.reshape(3, 3))
TypeError: mju_mat2Quat(): incompatible function arguments. The following argument types are supported:
    1. (quat: numpy.ndarray[numpy.float64[4, 1], flags.writeable], mat: numpy.ndarray[numpy.float64[9, 1]]) -> None

Invoked with: array([4.63659313e-310, 0.00000000e+000, 4.63659307e-310, 6.93593988e-310]), array([[-6.57059795e-09,  9.97564051e-01,  6.97564673e-02],
       [ 1.00000000e+00,  6.65583389e-09, -9.89480879e-10],
       [-1.45135801e-09,  6.97564673e-02, -9.97564051e-01]])

```

```
(botenv) jojo@jojo-System-Product-Name:~/Documents/ZMAI/IS_Bot$ python3 is_main.py --sim --show-images
正在创建模拟环境，模型路径: models/kinova_gen3/scene_2f85.xml
Traceback (most recent call last):
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_main.py", line 122, in <module>
    main(parser.parse_args())
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_main.py", line 92, in main
    env = KinovaMujocoEnv(show_images=args.show_images)
          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_mujoco_env.py", line 231, in __init__
    super().__init__(model_path=model_path, 
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
TypeError: MujocoEnv.__init__() got an unexpected keyword argument 'model_path'

```

18:21
```
Exception in thread Thread-2 (_physics_loop):
Traceback (most recent call last):
  File "/home/jojo/anaconda3/lib/python3.12/threading.py", line 1073, in _bootstrap_inner
    self.run()
  File "/home/jojo/anaconda3/lib/python3.12/threading.py", line 1010, in run
    self._target(*self._args, **self._kwargs)
  File "/home/jojo/Documents/ZMAI/botenv/lib/python3.12/site-packages/mujoco/viewer.py", line 306, in _physics_loop
    mujoco.mj_step(m, d)
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_mujoco_env.py", line 211, in control_callback
    mujoco.mju_mat2Quat(self.site_quat, self.site_xmat.reshape(3, 3))
TypeError: mju_mat2Quat(): incompatible function arguments. The following argument types are supported:
    1. (quat: numpy.ndarray[numpy.float64[4, 1], flags.writeable], mat: numpy.ndarray[numpy.float64[9, 1]]) -> None

Invoked with: array([4.63849184e-310, 0.00000000e+000, 4.63849216e-310, 6.92022794e-310]), array([[-6.57059795e-09,  9.97564051e-01,  6.97564673e-02],
       [ 1.00000000e+00,  6.65583389e-09, -9.89480879e-10],
       [-1.45135801e-09,  6.97564673e-02, -9.97564051e-01]])
状态更新: episode_started
遥操作控制器状态: primary_device_id=None
targets_initialized=False
遥操作策略初始化完成
遥操作策略已初始化
重置环境...
状态更新: episode_ended
状态更新: reset_env
状态更新: episode_started
状态更新: episode_ended
状态更新: reset_env
```

forgot the show images and connect the link
```
(botenv) jojo@jojo-System-Product-Name:~/Documents/ZMAI/IS_Bot$ python3 is_main.py --sim
正在创建模拟环境，模型路径: models/kinova_gen3/scene_2f85.xml
创建遥操作策略...
初始化遥操作策略...
启动服务器于 192.168.1.108:5000
正在初始化遥操作策略...
 * Serving Flask app 'is_kinova_teleop'
 * Debug mode: off
Process Process-1:
Traceback (most recent call last):
  File "/home/jojo/anaconda3/lib/python3.12/multiprocessing/process.py", line 314, in _bootstrap
    self.run()
  File "/home/jojo/anaconda3/lib/python3.12/multiprocessing/process.py", line 108, in run
    self._target(*self._args, **self._kwargs)
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_mujoco_env.py", line 270, in physics_loop
    sim = MujocoSim(self.mjcf_path, self.command_queue, self.shm_state, show_viewer=self.show_viewer)
          ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_mujoco_env.py", line 187, in __init__
    self.base_height = self.model.body('gen3/base_link').pos[2]
                       ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
KeyError: "Invalid name 'gen3/base_link'. Valid names: ['base', 'base_link', 'bracelet_link', 'forearm_link', 'half_arm_1_link', 'half_arm_2_link', 'left_coupler', 'left_driver', 'left_follower', 'left_pad', 'left_silicone_pad', 'left_spring_link', 'right_coupler', 'right_driver', 'right_follower', 'right_pad', 'right_silicone_pad', 'right_spring_link', 'shoulder_link', 'spherical_wrist_1_link', 'spherical_wrist_2_link', 'world']"
^CTraceback (most recent call last):
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_main.py", line 122, in <module>
    main(parser.parse_args())
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_main.py", line 105, in main
    policy.reset()
  File "/home/jojo/Documents/ZMAI/IS_Bot/is_kinova_teleop.py", line 180, in reset
    time.sleep(0.01)
KeyboardInterrupt

```

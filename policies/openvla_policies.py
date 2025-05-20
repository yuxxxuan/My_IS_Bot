import requests
import json_numpy
json_numpy.patch()
import numpy as np
from mujoco_env import MujocoEnv
import matplotlib.pyplot as plt  # 导入matplotlib
import os
import os.path
import cv2
import time 

class OpenVLAPolicy:
    def __init__(self, server_url= "http://192.168.3.101:9000",env=None):
        self.server_url = server_url
        self.default_instruction = "Pick up the orange and put it in the drawer."
        #Pick up the orange and put it in the drawer.
        self.unnorm_key = "lerobot_dataset"
        self.env = env

        # === 预加载图片路径并初始化索引 ===
        self.images_folder = '/home/fine/Documents/wge/openvla/images_2.5Hz'
        self.image_files = sorted([
            f for f in os.listdir(self.images_folder)
            if f.lower().endswith(('.png', '.jpg', '.jpeg'))
        ])
        if not self.image_files:
            raise FileNotFoundError(f"No image files found in {self.images_folder}")
        self.image_index = 0  # 当前图片索引       

    def reset(self):
        pass  # 保持空即可

    def step(self,observation):
        start_time = time.time()  # === 记录开始时间 ===

        ##-===------------image from training --------====------
       
        # image_file = self.image_files[self.image_index]
        # image_path = os.path.join(self.images_folder, image_file)
        # print(f"[Server] Reading image: {image_path}")

        # # 增加索引（循环）
        # self.image_index = (self.image_index + 1) % len(self.image_files)

        # # 读取图片
        # frame = cv2.imread(image_path)
        # # BGR -> RGB
        # image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        ##---====----------image from mujoco---------=====------
        
        image = observation.get("image", None)
        if image is None:
            print("No image found in observation!")
            return None
        if not isinstance(image, np.ndarray):
            image = np.array(image, dtype=np.uint8)

        # # ==============可视化图像================
        # plt.imshow(image)
        # plt.axis('off')  # 不显示坐标轴
        # plt.show()  # 显示图像

        # 构造 payload
        payload = {
            "image": image,
            "instruction": self.default_instruction,
            "unnorm_key": self.unnorm_key,
        }

        try:
            response = requests.post(f"{self.server_url}/act", json=payload)
            response.raise_for_status()
            action = response.json()
            print("Action received:", action)

            # === 记录结束时间并计算耗时 ===
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"[Time] Inference step took {elapsed_time:.3f} seconds ({1/elapsed_time:.2f} Hz)")

            # 原始角度向量（单位：度）
            raw_degrees = action[:7]# 提取前7个量作为 qpos

            # 映射到 [-π, π]
            qpos = np.deg2rad((raw_degrees + 180) % 360 - 180)
            
            print("qpos=",qpos)
            ctrl_gripper = action[7]# 最后一个量作为ctrl_gripper

            # 调用环境控制机械臂的接口
            self.env.send_arm_qpos_command(qpos=qpos, ctrl_gripper=ctrl_gripper)
            return action

        except Exception as e:
            print("Action request failed:", e)
            return None
        
    

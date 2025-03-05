# Author: ZMAI
# Date: 2025-03-06
import time
import numpy as np
from mujoco_env import MujocoEnv
from kinova import TorqueControlledArm
from constants import POLICY_CONTROL_PERIOD

class KinovaSimEnv(MujocoEnv):
    # 在MujocoEnv初始化时需要传入model_path参数
    def __init__(self, render_images=True, show_viewer=True, show_images=False):
        self.mjcf_path = 'models/kinova_gen3/scene_2f85.xml'
        super().__init__(model_path=self.mjcf_path, render_images=render_images, 
                         show_viewer=show_viewer, show_images=show_images)
    def get_obs(self):
        arm_quat = self.shm_state.arm_quat[[1, 2, 3, 0]]  # (w, x, y, z) -> (x, y, z, w)
        if arm_quat[3] < 0.0:  # Enforce quaternion uniqueness
            np.negative(arm_quat, out=arm_quat)
        obs = {
            'arm_pos': self.shm_state.arm_pos.copy(),
            'arm_quat': arm_quat,
            'gripper_pos': self.shm_state.gripper_pos.copy(),
        }
        if self.render_images:
            for shm_image in self.shm_images:
                obs[f'{shm_image.camera_name}_image'] = shm_image.data.copy()
        return obs

class KinovaRealEnv:
    def __init__(self):
        self.arm = TorqueControlledArm()
        self.arm.init_cyclic(self._control_callback)
        self.last_obs = None
        
    def _control_callback(self, command):
        if command is not None:
            if 'arm_pos' in command and 'arm_quat' in command:
                self.arm.move_to_pose(command['arm_pos'], command['arm_quat'])
            if 'gripper_pos' in command:
                self.arm.set_gripper_position(command['gripper_pos'])
    
    def reset(self):
        self.arm.move_to_home_position()
        self.last_obs = self.get_obs()
        return self.last_obs
    
    def get_obs(self):
        arm_pos, arm_quat = self.arm.get_end_effector_pose()
        gripper_pos = self.arm.get_gripper_position()
        obs = {
            'arm_pos': arm_pos,
            'arm_quat': arm_quat,
            'gripper_pos': gripper_pos,
        }
        self.last_obs = obs
        return obs
    
    def step(self, action):
        self._control_callback(action)
        return self.get_obs()
    
    def close(self):
        self.arm.disconnect()

if __name__ == '__main__':
    # Test simulation environment
    env = KinovaSimEnv(show_images=True)
    try:
        while True:
            env.reset()
            for _ in range(100):
                action = {
                    'arm_pos': 0.1 * np.random.rand(3) + np.array([0.55, 0.0, 0.4]),
                    'arm_quat': np.random.rand(4),
                    'gripper_pos': np.random.rand(1),
                }
                env.step(action)
                obs = env.get_obs()
                print([(k, v.shape) if v.ndim == 3 else (k, v) for (k, v) in obs.items()])
                time.sleep(POLICY_CONTROL_PERIOD)  # Note: Not precise
    finally:
        env.close()
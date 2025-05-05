import os,sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
from robot_controller.gen3.kinova import TorqueControlledArm

arm = TorqueControlledArm()
arm.retract()
arm.disconnect()
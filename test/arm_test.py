# Gravity compensation
import time
import os,sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
from robot_controller.gen3.kinova import TorqueControlledArm, grav_comp_control_callback

arm = TorqueControlledArm()
try:
    arm.init_cyclic(grav_comp_control_callback)
    while arm.cyclic_running:
        time.sleep(0.01)
except KeyboardInterrupt:
    arm.stop_cyclic()
    arm.disconnect()
import time
from kinova import TorqueControlledArm, grav_comp_control_callback

arm = TorqueControlledArm()
try:
    arm.init_cyclic(grav_comp_control_callback)
    while arm.cyclic_running:
        time.sleep(0.01)
except KeyboardInterrupt:
    arm.stop_cyclic()
    arm.disconnect()
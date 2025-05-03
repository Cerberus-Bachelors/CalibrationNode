import odrive
from odrive.enums import *
import time

odrive_serials = {
            "20763599524B": "front_hip",
            "2081356E524B": "front_thigh",
            "205B39814D4D": "front_calf",
            "208339814D4D": "rear_hip",
            "345735573033": "rear_thigh",
            "346035533033": "rear_calf",
        }

def main():
    for serial in odrive_serials:
        odrv = odrive.find_any(serial_number=serial, timeout=10)
        odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

if __name__ == '__main__':
    main()
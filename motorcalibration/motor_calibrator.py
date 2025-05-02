import odrive
from odrive.enums import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

from cerberus_msgs.msg import EspCommands

class motorCalibrator(Node):
    def __init__(self):
        super().__init__(
            'motor_calibrator',
        )

        self.odrive_serials = {
            "20763599524B": "front_hip",
            "2081356E524B": "front_thigh",
            "205B39814D4D": "front_calf",
            "208339814D4D": "rear_hip",
            "345735573033": "rear_thigh",
            "346035533033": "rear_calf",
        }

        self.hip_serials = [s for s, l in self.odrive_serials.items() if 'hip' in l]
        self.thigh_serials = [s for s, l in self.odrive_serials.items() if 'thigh' in l]
        self.calf_serials = [s for s, l in self.odrive_serials.items() if 'calf' in l]

        #self.get_logger().info(f'Loaded ODrive serials: {self.odrive_serials}')

        self.subscription = self.create_subscription(
            EspCommands,
            '/esp_commands',
            self.listenerCallback,
            10)
        self.subscription
        self.get_logger().info('Motor Calibrator node has been started.')
    
    def listenerCallback(self, msg):
        if msg.calibrate:
            self.get_logger().info('Received calibration trigger')
            self.connect_to_odrives()
            self.calibrate_all_motors()

    def connect_to_odrives(self):
        self.get_logger().info('Searching for ODrives...')
        self.connected_odrives = {}
        self.hip_odrives = []
        self.thigh_odrives = []
        self.calf_odrives = []

        for serial, label in self.odrive_serials.items():
            self.get_logger().info(f'Trying to connect to {label} (SN: {serial})...')
            try:
                odrv = odrive.find_any(serial_number=serial, timeout=10)
                if odrv:
                    self.connected_odrives[serial] = odrv
                    self.get_logger().info(f'Connected to {label} (SN: {serial})')
                    if 'hip' in label:
                        self.hip_odrives.append((serial, odrv))
                    elif 'thigh' in label:
                        self.thigh_odrives.append((serial, odrv))
                    elif 'calf' in label:
                        self.calf_odrives.append((serial, odrv))
                else:
                    self.get_logger().warn(f'ODrive not found: {label} (SN: {serial})')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to {label} (SN: {serial}): {e}')

    def calibrate_odrive_group(self, group_name, odrive_list):
        self.get_logger().info(f'Calibrating: {group_name}')

        for serial, odrv in odrive_list:
            for axis_name in ['axis0', 'axis1']:
                axis = getattr(odrv, axis_name)

                axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                time.sleep(7)

        all_idle = False
        while not all_idle:
            all_idle = True
            for serial, odrv in odrive_list:
                for axis_name in ['axis0', 'axis1']:
                    axis = getattr(odrv, axis_name)
                    if axis.current_state != AXIS_STATE_IDLE:
                        all_idle = False
            time.sleep(0.1)
        
        for serial, odrv in odrive_list:
            existing_error = False
            joint_label = self.odrive_serials[serial]
            for axis_name in ['axis0', 'axis1']:
                axis = getattr(odrv, axis_name)
                if axis.motor.error or axis.encoder.error or axis.error:
                    existing_error = True
                    self.get_logger().warn(
                        f'Errors on {joint_label}, {axis_name}: axis={axis.error}, motor={axis.motor.error}, encoder={axis.encoder.error}'
                    )
            
            if existing_error:
                self.get_logger().warn(f'Calibration error in {group_name}. Aborting sequence.')
                self.calibration_failed = True
                return
            
    def jog_motors(self, group_name, odrive_list, position):
        self.get_logger().info(f'Jogging {group_name} motors')

        
        for serial, odrv in odrive_list:
            label = self.odrive_serials.get(serial, serial)

            if "hip" in label:
                if "front" in label:
                    pos0 = position
                    pos1 = -position
                elif "rear" in label:
                    pos0 = -position
                    pos1 = position

            elif "thigh" in label:
                pos0 = position
                pos1 = -position

            elif "calf" in label:
                pos0 = position
                pos1 = -position

            # Axis 0
            axis0 = odrv.axis0
            axis0.encoder.set_linear_count(0)
            axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axis0.controller.input_pos = pos0
            self.get_logger().info(f'{label} axis0 moved to {pos0}')

            # Axis 1
            axis1 = odrv.axis1
            axis1.encoder.set_linear_count(0)
            axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            axis1.controller.input_pos = pos1
            self.get_logger().info(f'{label} axis1 moved to {pos1}')

        time.sleep(2)


    def calibrate_all_motors(self):
        self.calibration_failed = False
        if not self.calibration_failed:
            self.get_logger().info('Starting calibration sequence')
            self.calibrate_odrive_group('hip', self.hip_odrives)
            self.jog_motors('hip', self.hip_odrives, 0.8)
            self.calibrate_odrive_group('thigh', self.thigh_odrives)
            self.jog_motors('thighs', self.thigh_odrives, 0.2)
            self.calibrate_odrive_group('calf', self.calf_odrives)
            self.jog_motors('calf', self.calf_odrives, 0.1)

        else:
            self.get_logger().error('Aborted calibration sequence')

        
def main(args=None):
    rclpy.init(args=args)
    node = motorCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


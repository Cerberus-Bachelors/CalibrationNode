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

        self.get_logger().info(f'Loaded ODrive serials: {self.odrive_serials}')

        self.subscription = self.create_subscription(
            EspCommands,
            '/esp_commands',
            self.listenerCallback,
            10)
        self.subscription
        self.get_logger().info('Motor Calibrator node has been started.')

        self.connect_to_odrives()
    
    def listenerCallback(self, msg):
        if msg.calibrate:
            self.get_logger().info('Received calibration trigger — starting calibration...')
            self.calibrate_all_motors()

    def connect_to_odrives(self):
        self.get_logger().info('Searching for ODrives...')
        self.connected_odrives = {}

        for serial,label in self.odrive_serials.items():
             self.get_logger().info(f'Trying to connect to ODrive: {label} (SN: {serial})...')
        try:
            odrv = odrive.find_any(serial_number=serial, timeout=10)
            time.sleep(2)
            if odrv:
                self.connected_odrives[serial] = odrv
                self.get_logger().info(f'Connected to {label} (SN: {serial})')
            else:
                self.get_logger().warn(f'ODrive not found: {label} (SN: {serial})')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to {label} (SN: {serial}): {e}')
        time.sleep(1)

    def calibrate_all_motors(self):
        #Add calibration shits

def main(args=None):
    rclpy.init(args=args)
    node = motorCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


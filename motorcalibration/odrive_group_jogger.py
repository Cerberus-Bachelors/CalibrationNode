import rclpy
from rclpy.node import Node
from cerberus_msgs.msg import JogCommand

import odrive
from odrive.enums import *

import time

class ODriveGroupJogger(Node):
    def __init__(self):
        super().__init__('odrive_group_jogger')

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


        self.subscription = self.create_subscription(
            JogCommand,
            '/jog_group',
            self.listener_callback,
            10
        )

        self.get_logger().info("ODrive Group Jogger started")

    def listener_callback(self, msg):
        group = msg.group_name.lower()
        position = msg.position

        if group == 'hip':
            serials = self.hip_serials
        elif group == 'thigh':
            serials = self.thigh_serials
        elif group == 'calf':
            serials = self.calf_serials
        else:
            self.get_logger().error(f"Invalid group: {group}")
            return

        self.get_logger().info(f"Jogging group '{group}' to position {position}")

        for serial in serials:
            label = self.odrive_serials.get(serial, serial)
            try:
                odrv = odrive.find_any(serial_number=serial, timeout=10)
                self.get_logger().info(f"Connected to {label}")

                # Determine jog direction per axis based on label and group
                if "hip" in label:
                    if "front" in label:
                        pos0 = position
                        pos1 = -position
                    elif "rear" in label:
                        pos0 = -position
                        pos1 = position
                    else:
                        pos0 = pos1 = 0.0  # fallback

                elif "thigh" in label or "calf" in label:
                    pos0 = position
                    pos1 = -position

                else:
                    self.get_logger().warn(f"Unknown label '{label}', skipping.")
                    continue

                # Axis 0
                axis0 = odrv.axis0
                axis0.encoder.set_linear_count(0)
                axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                axis0.controller.input_pos = pos0
                self.get_logger().info(f"{label} axis0 moved to {pos0}")

                # Axis 1
                axis1 = odrv.axis1
                axis1.encoder.set_linear_count(0)
                axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                axis1.controller.input_pos = pos1
                self.get_logger().info(f"{label} axis1 moved to {pos1}")

            except Exception as e:
                self.get_logger().error(f"Could not connect to {label}: {e}")

        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = ODriveGroupJogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


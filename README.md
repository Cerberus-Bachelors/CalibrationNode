# motorcalibration

**motorcalibration** is a ROS 2 package for calibrating and managing ODrive-controlled motors in multi-joint robotic systems. It includes utilities to calibrate, configure, idle, enable, and jog motor groups.

## Overview of Commands

- **`calibrate_motors`**: Calibrates all joints using the connected ODrive boards.
  ```bash
  ros2 run motorcalibration calibrate_motors
  ```

- **`config_drivers`**: One-time setup script to configure all ODrive motor drivers.
  ```bash
  ros2 run motorcalibration config_drivers
  ```

- **`idle_drivers`**: Puts all drives into idle mode.
  ```bash
  ros2 run motorcalibration idle_drivers
  ```

- **`closed_loop_drivers`**: Enables all drives in closed-loop control.
  ```bash
  ros2 run motorcalibration closed_loop_drivers
  ```

- **`jog_group`**: Incrementally moves a group of motors by a specified delta.
  ```bash
  ros2 run motorcalibration jog_group
  ```
## Example: Test Commands to publish

You can trigger calibration using a ROS 2 topic:

```bash
ros2 topic pub /esp_commands cerberus_msgs/EspCommands "{calibrate: true, startup: false, shutdown: false}" -1
```
Command to use jog program:

```bash
  ros2 topic pub /jog_group cerberus_msgs/JogCommand "{group_name: 'calf', position: -2}" -1
```
> Note: `position` here represents **delta movement**, not an absolute target.

## On robot startup:
When starting the robot calibrate should be run. This is setup over esp_commands message which allows an esp32 to send an output to the program to initialise calibrate sequence. The calibrate program should be started on robot power up and will sit and wait until calibrate command is recieved.



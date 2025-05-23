# Omni-Wheels-Triangle-Robot
A ROS 2 and Arduino-based control system for a 3-wheel omni robot. A ROS node listens to /cmd_vel, calculates wheel speeds using omni kinematics, and sends them to Arduino via serial. Arduino reads the speeds and controls motors using PWM. Ideal for mobile robots with omnidirectional movement.
# Omni-Wheel ROS2 + Arduino Control

A ROS 2 and Arduino-based system for controlling a 3-wheel omni-directional robot.

##  Overview

- ROS 2 node subscribes to `/cmd_vel`
- Calculates individual wheel speeds using omni kinematics
- Sends speeds to Arduino via serial
- Arduino controls 3 DC motors using PWM

## 🛠 ROS 2 Setup

```bash
cd ~/ros_ws/src
git clone https://github.com/yourusername/omni-ros2-arduino.git
cd ..
colcon build --packages-select control_pkg
source install/setup.bash
ros2 run control_pkg omni_serial_node

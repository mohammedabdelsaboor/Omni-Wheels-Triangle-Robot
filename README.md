# Omni-Wheels-Triangle-Robot
A ROS 2 and Arduino-based control system for a 3-wheel omni robot. A ROS node listens to /cmd_vel, calculates wheel speeds using omni kinematics, and sends them to Arduino via serial. Arduino reads the speeds and controls motors using PWM. Ideal for mobile robots with omnidirectional movement.

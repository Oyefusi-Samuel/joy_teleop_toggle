![image](https://github.com/user-attachments/assets/1bd9bf31-95b7-4373-9061-cdef64b4b16a)

![image](https://github.com/user-attachments/assets/43c52ff0-15bb-43d6-b516-1b3c2463687e)  ![image](https://github.com/user-attachments/assets/cbc20e44-9ffc-4a4a-b2ff-ba822cb9cebd)  ![image](https://github.com/user-attachments/assets/005a1434-1fee-4fbb-9ddf-41639b779686)
![image](https://github.com/user-attachments/assets/958560e6-67a2-41d0-9b7c-59c794101e9a)  ![image](https://github.com/user-attachments/assets/3444dcb2-10ac-40ec-8430-d80d7704de0d)




# joy_teleop_toggle
A simple ROS 2 Python node for joystick teleoperation with toggle enable/disable and dynamic velocity scaling, optimized for DualShock 4 and any robot joystick controllers for mobile robotics.

Features include:

- Toggle teleoperation mode ON/OFF with a button press.
- Use the left joystick to control **linear velocity** (forward/backward).
- Use the right joystick to control **angular velocity** (left/right turns).
- Adjust linear and angular velocity scales dynamically with controller buttons (L2/R2 for linear speed, L1/R1 for angular speed).
- Debounced button presses for smooth velocity scaling.
- Velocity levels persist while teleop is enabled.
- Publishes `geometry_msgs/Twist` commands to `cmd_vel` topic.
- Automatically starts the `joy_node` driver as a subprocess.

---

## Installation

1. Clone this repository into your ROS2 workspace `src` folder:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/joy_teleop_toggle.git

2. Build the package with colcon:

   ```bash
cd ~/ros2_ws
colcon build --packages-select joy_teleop_toggle --symlink-install

3. Source your workspace:

source ~/ros2_ws/install/setup.bash

Source your workspace:

    source ~/ros2_ws/install/setup.bash

Usage

Connect your DualShock 4 controller (wired or wireless) and run:

ros2 run joy_teleop_toggle joy_teleop_toggle

    Press the Circle button (Button 1) to toggle teleoperation ON/OFF.

    Use the left joystick (vertical axis) to move forward/backward.

    Use the right joystick (horizontal axis) to turn left/right.

    Press R2 (Button 7) to increase linear speed.

    Press L2 (Button 6) to decrease linear speed.

    Press R1 (Button 5) to increase angular speed.

    Press L1 (Button 4) to decrease angular speed.

Current velocity scales will be printed in the terminal when changed.

Button Mapping Summary
Function	Button / Axis
Toggle teleop	Circle (Button 1)
Increase linear vel	R2 (Button 7)
Decrease linear vel	L2 (Button 6)
Increase angular vel	R1 (Button 5)
Decrease angular vel	L1 (Button 4)
Linear velocity	Left joystick vertical (axes[1])
Angular velocity	Right joystick horizontal (axes[3])

# Dependencies

    ROS 2 (tested on Humble/Foxy/Galactic)

    joy package (for joystick driver)

Install the joystick package if not installed:

sudo apt install ros-<ros2-distro>-joy

# License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/Oyefusi-Samuel/joy_teleop_toggle/blob/main/LICENSE) file for details.

# Contributions

Contributions and improvements are welcome! Please open issues or pull requests on [GitHub.](https://github.com/).

# Acknowledgements

Inspired by ROS [teleop_twist_joy](https://github.com/ros-drivers/joystick_drivers.git) and the Open robotics community efforts to improve robot joystick teleoperation.

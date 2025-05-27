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


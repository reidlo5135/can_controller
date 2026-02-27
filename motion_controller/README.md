## MotionController
### OutLine 
Ros2 WmMotionController is a program that receives calculated result values and controls movement to the robot and delivers the result values through can communication.

CAN (Controller Area Network) is a communication protocol used in control systems such as cars and robots, suitable for fast data transmission and ensuring reliability. In ROS 2, CAN communication is supported through the can_msgs package.

First, obtain the cmd_vel value published by ros2 and calculate it according to the movement.

Next, the nodes that control the robot transmit accordingly to the command protocol over CAN communication.

### Dev
* Ubuntu 22.04
* Humble

### Network dev
* CAN
* DDS

### Guide
* Package Name : wm_motion_controller
* Node Name : wm_motion_controller_node

### Compile
```
    ${packages_dir} / colcon build --symlink-install
```

### Parameter 
* wheel.size [float]: Car wheel size. (m) - default (0.5078) 
* wheel.max_angle [float]: Maximum car rotation angle (Degree) - default (30.0)
* wheel.base_length [float] : the distance between the tires and the tires (m) - default (1.15)
* robot.max_speed [float] : Maximum car speed (m/s) - default (3.0)
* imu_correction [float] : Position correction due to attachment of imu sensor or change of reference point (Degree) - default (90)
```
    robot:
        max_speed: 5.5
    wheel:
        size: 0.5078
        max_angle: 30.0
        base_length: 1.15
    imu_correction: 90.0

```
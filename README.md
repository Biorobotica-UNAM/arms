# arms
This repository provides a ROS2 package to control the Justina's robotic arms and grippers.

## Configuration
The arms have 4-DOF using the Dynamixel motors in the following configuration:

* *Shoulder*: 2 x MX-106
* *Arm*: 1 x MX-64
* *Elbow*: 1 x MX-106
* *Wrist*: 1 x MX-64
* *Jaws*: 2 x MX-28

## Description
The node receives the position of each motor in radians and converts it into bits. Those bits are written to the motor's registers using the Dynamixel SDK. To install this dependency run the script `scripts/setup_justina.sh` in Justina repository. Or you can run the following command in terminal:

```bash
sudo apt install ros-jazzy-dynamixel-sdk
```

Additionally, it pusblishes the JointStates of the arm to be visualized in RViz.

## Interfaces
There are two topics to command the position in radians to the motors. Those are:

* `/goal_pose`
* `/goal_gripper`

The type of message for both topics is `std_msgs::msg::Float64MultiArray`.

At the same time, the node is publishing to the following topics:

* `/joint_states` [`sensor_msgs::msg::JointState`] - Publishes the positions of the joints.
* `/current_pose` [`std_msgs::msg::Float64MultiArray`] - Publishes the current positions of the arm's motor.
* `/current_gripper` [`std_msgs::msg::Float64`] - Pusblishes the current position of the gripper.
* `/goal_reached` [`std_msgs::msg::Bool`] - Indicates whether the position in every motor was reached or not.
* `/voltage` [`std_msgs::msg::Float64`] - Indicates the voltage level of the motors.
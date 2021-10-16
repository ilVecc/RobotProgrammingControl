# Robot Programming & Control - High-level homework

As part of the RPC course, students were asked to work on URDF files, FK/IK solvers and the MoveIt! architecture.
This repository contains my scripts for those exercises and the instructions to run them.

## Homework 1: URDF and FK/IK
We were asked to write the URDF file of a quasi-standard SCARA robot (an extra wrist joint is added at the end of the elbow link).
The workspace is thus the usual SCARA workspace, very similar to the one shown below.
![SCARA workspace](https://www.dailyautomation.sk/wp-content/uploads/2016/05/Scara-roboty-workspace.-GIF.gif)
Due to it's structure, we have an elbow singularity for most of the poses.


The `homework_scara_robot.urdf.xacro` file contains the URDF file of the given robot. 

The `kinematics.py` file shows an example of FK/IK for the robot using the DH table and homogeneous matrices.

The `ros_kinematics.py` file compares ROS TF and the previous manual computation, showing the same results.


Python2 and the `numpy`, `numpy-quaternion` packages are required.


## Homework 2: MoveIt! planning
We were asked to implement some basic trajectory planning using the MoveIt! architecture. MoveIt! can be installed on ROS-Melodic simply with `sudo apt install ros-melodic-moveit`, so it's not required to manually compile it.
On the other hand, the `universal_robot` bundle must be manually cloned and built in your catkin workspace. 

The `moveit_ur5.py` file contains each sub-task, which are
- go to a specific pose
- go to a specific joint state
- execute a line trajectory
- execute a trajectory with multiple waypoints
- execute a XY-plane line trajectory
- show collision avoidance
- show collision avoidance with an object attached to the robot hand

In order to have meaningful results in the last two tasks with the OMPL algorithm, the number of maximum attempts is set to 10.

Python2 is required.

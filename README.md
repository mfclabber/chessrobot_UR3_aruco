# chessrobot_UR3_aruco
UR3 robot playing chess with aruco marks by ROS

## Setup

### Chessboard

The cells of the chessboard have sides of 5 cm; and are labeled as usual with numbers (the rows) and letters (the columns):

![image](https://github.com/mfclabber/chessrobot_UR3_aruco/assets/118126641/09072c99-fb1b-4d47-84ab-5e3a2cd34894)


There are 8 aruco markers on the chessboard, with labels 100 to 107, that will be used to calibrate the camera pose. Their size is 26 x 26 mm.

The world reference frame is located at the center of the chessboard.


### Provided packages

The following packages grouped in the meta-repository are provided:

* UR3-IK: ROS server that ofers the Inverse Kinematics of the UR3 robot as a service.

* kinenik: Library to compute the Inverse Kinematics of any robot from Universal Robots (UR3, UR5, UR10, UR3e, UR5e, UR10e).

* Universal_Robots_ROS_Driver: Driver enabling ROS operation of UR robots.

* fmaunch_universal_robot: ROS support for the universal robots.

* aruco_ros: Software package and ROS wrappers of the Aruco Augmented Reality marker detector library.

* aruco_broadcaster: Project to easily configure and publish the tf of the aruco frames detected.

* downward_ros: ROS wrapper to the Fast Downward planning system.

* gazebo_ros_link_attacher: Utility to attach/detach models in Gazebo.

* robotiq: ROS-Industrial robotiq meta-package.

* robitcsgroups_gazebo_plugin: Gazebo plugin needed for the simulation of the gripper.

* tablesens: Package for the calibration of the camera.

* test_hardware: Package to test the devices.

![image](https://github.com/mfclabber/chessrobot_UR3_aruco/assets/118126641/8db2887f-064a-4c2c-8504-ac7a8e9a7b37)

![image](https://github.com/mfclabber/chessrobot_UR3_aruco/assets/118126641/2924c837-99a3-4c7e-870f-1f6f54d59392)

![image](https://github.com/mfclabber/chessrobot_UR3_aruco/assets/118126641/e3163dbb-033e-4f07-9e2d-895e50c0f724)

[chess.webm](https://github.com/mfclabber/chessrobot_UR3_aruco/assets/118126641/a9a432d4-3fea-4eb4-ad58-7604b48f7522)



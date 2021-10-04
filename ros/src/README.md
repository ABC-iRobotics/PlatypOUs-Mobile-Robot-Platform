# PlatypOUs ROS workspace
This document contains the descriptions of the packages in the workspace, as well as the instructions on using the robot in a simulated environment.

## Packages
The packages can contain source code, launch and parameter files, message definitions, URDF, etc.

#### platypous_driver
This package contains the driver class and the ROS node for controlling the ODrive motor controller board. It is based on the ODrive firmware version v0.5.1.

#### platypous_gazebo
This package contains the URDF for the robot to use in the Gazebo simulator. It also has some premade worlds, rviz preset, and extra stuff needed for the simulation.

#### platypous_launch
In this package, there are the launch files for the ROS nodes used by the robot. It also contains the YAML files for the parameters. There are separate launch files for individual parts of the system, as well as different configurations for system startup.

#### platypous_msgs
Contains custom message definitions used by some nodes in the system.

#### platypous_status
This package contains the sources of custom nodes for the collection of robot status data, as well as the node converting the map to image format.

#### eeg_node
Node for using the robot with the MindRove EEG headset, getting commands from MQTT topics, and publishing velocity commands to the motor controller based on the information.

## Simulation

This guide describes how to use the simulated PlatypOUs robot on your local machine.

### Requirements

- Ubuntu 20.04 Focal Fossa or Debian 10 Buster
- ROS Noetic
- Gazebo 11
- other ROS packages: robot_localization, gmapping, twist_mux, move_base

### Instructions

#### 1. Install dependencies
If you do not have ROS installed, follow the instructions at http://wiki.ros.org/noetic/Installation.

After ROS is installed, install the other needed ROS packages if not installed already:
```
sudo apt install ros-noetic-robot-localization ros-noetic-gmapping ros-noetic-twist-mux ros-noetic-move-base
```

#### 2. Build the workspace
In the cloned or downloaded repository directory, build the workspace by running the *build_workspace* script.
```
./build_workspace
```

#### 3. Start the simulation
The simulated environment can be launched by one of the following scripts.

Basic configuration, only with essential features to move the robot:
```
./start_simulation_basic
```

Full functionality *without* the web server and GUI:
```
./start_simulation_full
```

Full functionality *with* the web server and GUI:
```
./start_simulation_full_web
```

### Using the simulated robot

The robot has 2 wheels that can be controlled to move the platform. The wheels produce encoder-like odometry. The robot has a LIDAR on the top, and a depth camera on the front.

- The robot can be moved by **geometry_msgs/Twist** messages on the **/platypous/cmd_vel** topic. This can come from any source, for example:
  - **teleop_twist_keyboard**: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/platypous/cmd_vel`
  - **rqt** robot steering plugin: run `rqt` and select Plugins->Robot tools->Robot Steering, then change the topic to **/platypous/cmd_vel**
  - game controller/joystick: `roslaunch platypous_launch joy_teleop.launch`. For this, the packages **joy**, **teleop_twist_joy**, and **twist_mux** have to be installed (`sudo apt install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-twist-mux`). If the joystick is *not* connected at **/dev/input/js0**, add the correct path to the launch command as the **dev** argument (`dev:=/path/to/joy`).

- Wheel encoder odometry is published to the **/differential_drive/odom** topic as **nav_msgs/Odometry**.

- An EKF state estimation node from **robot_localization** publishes the **odom->base_link** transform based on the wheel odometry.

- The laser scan from the LIDAR is published to the **/laser/scan** topic, as **sensor_msgs/LaserScan**.

- In the empty world, the **odom** frame is fixed to the **map** frame by a static transform. In the office environment, **gmapping** is started automatically, and publishes the **map->odom** transform. It also publishes the created map to the **/map** topic as **nav_msgs/OccupancyGrid**.

- The camera color image is published to the **/depth_camera/color/image_raw** topic, the depth image to the **/depth_camera/depth/image_raw** topic, both as **sensor_msgs/Image**. The depth camera point cloud is published to the **/depth_camera/depth/points** topic as **sensor_msgs/PointCloud2**.

- To view the state of the robot, and the result of the SLAM and the odometry, run `roslaunch platypous_gazebo rviz_slam.launch` for a premade RVIZ setup.

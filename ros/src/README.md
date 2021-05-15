# Simulation

This guide describes how to use the simulated PlatypOUs robot on your local machine.

## Requirements

- Ubuntu 20.04 Focal Fossa or Debian 10 Buster
- ROS Noetic
- Gazebo 11
- other ROS packages: robot_localization, gmapping

## Instructions

#### 1. Clone the repository or download and unzip it.
```bash= !
git clone https://github.com/ABC-iRobotics/PlatypOUs-Mobile-Robot-Platform.git
```

#### 2. Navigate into the *ros* directory.
```bash= !
cd PlatypOUs-Mobile-Robot-Platform/ros
```

#### 3. Build the workspace.
```bash= !
catkin_make
```

#### 4. Source the setup script.
```bash= !
source devel/setup.bash
```
- Note: to be able to use the workspace, you have to source this script every time you open a new terminal, or add `source ~/PlatypOUs-Mobile-Robot-Platform/ros/devel/setup.bash` to the end of the **~/.bashrc** file.

#### 5. Install needed packages
```bash= !
sudo apt install ros-noetic-robot-localization ros-noetic-gmapping
```

#### 6. Launch the simulation
- Empty world: `roslaunch platypous_gazebo empty.launch`
- Office (with SLAM): `roslaunch platypous_gazebo office.launch`

After that, Gazebo should start with the robot spawned in the selected environment.

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
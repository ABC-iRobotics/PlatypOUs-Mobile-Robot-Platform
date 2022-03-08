# PlatypOUs ROS workspace

## Packages

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


## Topics
The robot has 2 wheels that can be controlled to move the platform. The wheels produce encoder odometry. The robot has a LIDAR on the top, and a depth camera on the front.

This is a list of the most important topics for using the robot, like data from the sensors, and movement control. The topic names and types are the same for the real and the simulated robot.

### Robot control
Topic: **/cmd_vel** ([**geometry_msgs/Twist**](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))

***You shouldn't directly publish to this topic***, as there is a [**twist_mux**](http://wiki.ros.org/twist_mux) node running, which is listening to several other topics and prioritize between them. The topics are, in priority order:
- **/cmd_vel/joy_teleop**: used by the joystick
- **/cmd_vel/web_teleop**: used by the web GUI teleop function
- **/cmd_vel/nav**: used by the navigation algorithm
- **/cmd_vel/other**: can be used for any other purpose

For manual control, you can use any of them, for example the **/cmd_vel/joy_teleop** even if you don't have a joystick connected. Some examples:
- game controller/joystick: `roslaunch platypous_launch joystick_teleop.launch`. For this, the packages [**joy**](http://wiki.ros.org/joy) and [**teleop_twist_joy**](http://wiki.ros.org/teleop_twist_joy) have to be installed (`sudo apt install ros-noetic-joy ros-noetic-teleop-twist-joy`). If the joystick is *not* connected at **/dev/input/js0**, add the correct path to the launch command as the **dev** argument (`dev:=/path/to/joy`).
- [**teleop_twist_keyboard**](http://wiki.ros.org/teleop_twist_keyboard): `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel/joy_teleop`
- **rqt** robot steering plugin: run `rqt` and select Plugins->Robot tools->Robot Steering, then change the topic to **/cmd_vel/joy_teleop**

If you use your own(or any other) control/navigation algorithm, you should use the **/cmd_vel/nav** topic, because then you can override its commands by the manual/web GUI controls.

### Wheel odometry
Topic: **/odometry/wheel** ([**nav_msgs/Odometry**](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))

Wheel odometry is the estimation of the robot's current position and velocity based on the turning of the wheels.

Also, the **odom->base_link** transform is published based on this. For more information about this, see https://www.ros.org/reps/rep-0105.html and http://wiki.ros.org/tf2.

### Laser scanner
Topic: **/scan** ([**sensor_msgs/LaserScan**](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))

The LIDAR is mounted on the top of the platform, and provides range data in all 360 degrees, up to 12 meters. It is used by the SLAM (Simultaneous Localization and Mapping) algorithms.

### Depth camera
The camera provides several different topics.

Color image topic: **/camera/color/image_raw** ([**sensor_msgs/Image**](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))

Depth image topic: **/camera/aligned_depth_to_color/image_raw** ([**sensor_msgs/Image**](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html))

Point cloud topic: **/camera/depth/color/points** ([**sensor_msgs/PointCloud2**](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html))

### SLAM
Map topic: **/map** ([**nav_msgs/OccupancyGrid**](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)).

Odometry position correction is published by the **map->odom** transform.

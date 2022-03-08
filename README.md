# PlatypOUs Mobile Robot Platform

Software repository of the PlatypOUs differential drive mobile robot platform.

## Repository structure
 - **ros**: ROS workspace environment
 - **web**: web server and GUI
 - **docker**: Docker specific files

## Using the repository

To use the repository, you should be familiar with the following:
 - basic Linux usage, like using the terminal, navigating in the directories, or running scripts. Here is a link that may be useful for starting: https://ubuntu.com/tutorials/command-line-for-beginners
 - basic ROS concepts, like the nodes, topics, messages, publishing, subscribing, ROS command line tools, roslaunch, rqt, rviz, tf, etc... Make sure you have read the following: http://wiki.ros.org/ROS/Introduction, http://wiki.ros.org/ROS/Tutorials
 - basic Python or C++ knowledge, if you want to make your own nodes or alter existing ones
 - basic JavaScript and Node.JS knowledge if you want to make something for the web interface
 - Git usage, if you want to contribute to the repository
 - Docker, if you want to use the simulation from Docker. The real robot also runs ROS in Docker. See: https://docs.docker.com/get-started/

The robot can be used in two ways: with the real hardware, or in the Gazebo simulator. Both can be used with ROS installed natively, or using Docker.

## Running the simulation
### Requirements
If you want to use natively installed ROS, make sure you have the following installed:
- Ubuntu 20.04 Focal Fossa (recommended) or Debian 10 Buster (either installed natively, or in a virtual machine). See: https://ubuntu.com/tutorials/install-ubuntu-desktop, https://itsfoss.com/install-linux-in-virtualbox/
- ROS Noetic desktop full version. See: http://wiki.ros.org/noetic/Installation
- Gazebo 11 (should be already installed if you installed ROS desktop full version)
- ROS packages: robot_localization, gmapping, twist_mux, move_base, global_planner, dwa_local_planner:
`sudo apt install ros-noetic-robot-localization ros-noetic-gmapping ros-noetic-twist-mux ros-noetic-move-base ros-noetic-global-planner ros-noetic-dwa-local-planner`
- Node.js for the web GUI server:
`
curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - &&
apt-get install -y nodejs
`

Alternatively, you can use Docker. See: https://docs.docker.com/get-docker/. This is only tested using the Linux version of Docker Engine. It may or may not work on Windows.

### Instructions
#### 1. Clone or download the repository
```
git clone https://github.com/ABC-iRobotics/PlatypOUs-Mobile-Robot-Platform.git
cd PlatypOUs-Mobile-Robot-Platform
```

If you want to use Docker, generate the Docker image:
```
./platypous_docker build
```


#### 2. Initialize and build
This will initialize the web app if not initialized, and build the ROS workspace.

You should also run this every time you make a change in one of the ROS packages.

With ROS installed:
```
./platypous build
```

Using Docker:
```
./platypous_docker run build
```


#### 3. Start the simulation
This will start the simulation with the default configuration, and the server for the web GUI.

With ROS installed:
```
./platypous start_sim
```

Using Docker:
```
./platypous_docker run start_sim
```

## Using the robot
After starting the simulation, you should see the simulator window with the robot in it. The web GUI is available using a browser by typing: `localhost:3000`

If you use Docker, you may need additional terminals to start programs in the ROS environment. You can achieve this by running the image in interactive mode using:
```
./platypous_docker run
```
This will give you a terminal inside the ROS container, where you can use ROS and PlatypOUs related commands.

For information on the ROS packages, nodes and topics, see the [PlatypOUs ROS workspace](ros/src/README.md).

For information on the web server and GUI, see the [PlatypOUs Web GUI](web/README.md).

## Issues
If you find any problems with building or using this repository, please open an issue on GitHub: https://github.com/ABC-iRobotics/PlatypOUs-Mobile-Robot-Platform/issues

More detailed documentation is under construction, feel free to ask/suggest anything. Any feedback is greatly appreciated.

# PlatypOUs Mobile Robot Platform

Software repository of the PlatypOUs differential drive mobile robot platform.

## Repository structure
 - **ros**: ROS workspace environment
 - **web**: web server and GUI
 - **docker**: Docker specific files

## Using the repository

The software can be used in two ways: with the Gazebo simulator on your own machine (either a natively installed, compatible OS, or a virtual machine), or on the real robot, where it uses Docker.

To use the repository, you should be familiar with the following:
 - basic Linux usage, like using the terminal, navigating in the directories, or running scripts. Here is a link that may be useful for starting: https://ubuntu.com/tutorials/command-line-for-beginners
 - basic ROS concepts, like the nodes, topics, messages, publishing, subscribing, ROS command line tools, roslaunch, rqt, rviz, tf, etc... Make sure you have read the following: http://wiki.ros.org/ROS/Introduction, http://wiki.ros.org/ROS/Tutorials
 - basic Python or C++ knowledge, if you want to make your own nodes or alter existing ones
 - basic JavaScript and Node.JS knowledge if you want to make something for the web interface
 - Git usage, if you want to contribute to the repository
 - Docker, if you want to use the real robot. See: https://docs.docker.com/get-started/ (not really necessary for usage, but useful if you intend to understand/modify docker related stuff)

## Running the simulation
### Requirements
Make sure you have the following installed:
- Ubuntu 20.04 Focal Fossa (recommended) or Debian 10 Buster (either installed natively, or in a virtual machine). See: https://ubuntu.com/tutorials/install-ubuntu-desktop, https://itsfoss.com/install-linux-in-virtualbox/
- ROS Noetic desktop full version. See: http://wiki.ros.org/noetic/Installation
- Gazebo 11 (should be already installed if you installed ROS desktop full version)
- ROS packages: robot_localization, gmapping, twist_mux, move_base:
`sudo apt install ros-noetic-robot-localization ros-noetic-gmapping ros-noetic-twist-mux ros-noetic-move-base`
- Node.js for the web GUI server:
`
curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - &&
apt-get install -y nodejs
`

### Instructions
#### 1. Clone or download the repository
```
git clone https://github.com/ABC-iRobotics/PlatypOUs-Mobile-Robot-Platform.git
```

#### 2. Initialize and build
In the cloned or downloaded repository directory, build the workspace by running:
```
cd PlatypOUs-Mobile-Robot-Platform
./platypous_sim build
```
This will initialize the web app with `npm ci` if not initialized, and build the ROS workspace with `catkin build`.

You should also run this every time you make a change in one of the ROS packages.

#### 3. Start the simulation
```
./platypous_sim start
```
This should start the simulation with the default configuration, and also the server for the web GUI, which you can open in a browser by typing in: `localhost:3000`

## Real robot
On the robot, the environment is built into a docker image by the command:
```
./platypous_robot build
```

The image can be started in a docker container in the current terminal:
```
./platypous_robot start_once
```

Or the docker container can be started in the background (and will restart automatically, even after computer restarts):
```
./platypous_robot start
```

This can be stopped by removing the container:
```
./platypous_robot stop
```

## Using the robot
For information on the ROS packages, nodes and topics, see the [PlatypOUs ROS workspace](ros/src/README.md).
For information on the web server and GUI, see the [PlatypOUs Web GUI](web/README.md).

## Issues
If you find any problems with building or using this repository, please open an issue on GitHub: https://github.com/ABC-iRobotics/PlatypOUs-Mobile-Robot-Platform/issues

More detailed documentation is under construction, feel free to ask/suggest anything. Any feedback is greatly appreciated.

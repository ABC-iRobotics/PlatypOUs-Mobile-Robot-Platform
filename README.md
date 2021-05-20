# PlatypOUs Mobile Robot Platform

ROS based software of the PlatypOUs differential drive mobile robot platform.

This repository has two main parts:

 - **ros**: the ROS workspace
 - **web**: the web server and the web GUI

### ROS workspace
The ROS workspace contains the packages needed for robot operation. This includes launch files, YAML files for the parameters, source of custom made nodes, etc.

The documentation for the ROS stuff can be found at [here](ros/src/README.md).

### WEB server and GUI
This section contains the source and documentation of the Node.js web server, and the graphical interface for the robot.

More detailed description can be found at [here](web/README.md).

## Using the repository

### Simulation
For using the simulated version of the robot on your computer, please see the [ROS workspace documentation](ros/src/README.md).

### Real robot
On the real robot, the software can be used with Docker. For building up the environment, the Dockerfile contains all necessary information. The building process can be started with the *build_docker_image* script:
```bash= !
./build_docker_image
```
This should generate the docker image with everything needed to run the robot.

When the building finishes, the image can be started using one of the available start scripts.

The *start_robot_basic* script starts the robot with only basic functionality, like the motor control, and teleoperation. The *start_robot_full* version starts the robot with all currently available features.
```bash= !
./start_robot_basic
```
or
```bash= !
./start_robot_full
```
For more information on the features, and the different start configurations, see the [ROS workspace documentation](ros/src/README.md).

The previous methods only start the image in the current terminal, and stops when it is closed, or interrupted with CTRL+C. To run the robot independently, there exists a version of the sripts with *_perm* after their names. These start the process in the background, and after that, it will also start automatically when the robot is turned on. Running any of the scripts will remove the already running container, and restart the process in the selected way.
```bash= !
./start_robot_basic_perm
```
or
```bash= !
./start_robot_full_perm
```

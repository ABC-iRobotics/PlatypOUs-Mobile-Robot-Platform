FROM ros:noetic

RUN apt-get update && \
    apt-get install -y \
        libusb-dev \
        python3-pip \
        git

RUN python3 -m pip install odrive

RUN mkdir -p /root/ros_ws/src && \
    cd /root/ros_ws/src && \
    git clone https://github.com/YDLIDAR/ydlidar_ros && \
    cd ydlidar_ros && \
    git checkout master && \
    cd ../.. && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make

RUN apt-get update && \
    apt-get install -y \
        ros-noetic-joy \
        ros-noetic-teleop-twist-joy \
        ros-noetic-twist-mux \
        ros-noetic-robot-localization \
        ros-noetic-gmapping

COPY /ros/src /root/ros_ws/src
WORKDIR /root/ros_ws
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

WORKDIR /
COPY /entrypoint.bash /
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]

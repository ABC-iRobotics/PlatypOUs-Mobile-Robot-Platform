FROM ros:noetic

RUN apt-get update && \
    apt-get install -y \
        libusb-dev \
        python3-pip

RUN python3 -m pip install odrive

RUN apt-get update && \
    apt-get install -y \
        ros-noetic-joy \
        ros-noetic-teleop-twist-joy \
        ros-noetic-twist-mux

COPY /ros/src /root/ros_ws/src
WORKDIR /root/ros_ws
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

WORKDIR /
COPY /entrypoint.bash /
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]

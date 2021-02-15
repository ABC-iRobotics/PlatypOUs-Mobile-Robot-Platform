FROM ros:noetic

COPY /ros /root/ros_ws
WORKDIR /root/ros_ws
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

RUN apt-get update && \
    apt-get install -y \
        libusb-dev \
        python3-pip \
        ros-noetic-joy \
        ros-noetic-teleop-twist-joy && \
    python3 -m pip install odrive

WORKDIR /
COPY /entrypoint.bash /
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]

FROM ros:noetic

RUN apt-get update && \
    apt-get install -y \
        libusb-dev \
        libusb-1.0-0-dev \
        python3-pip \
        python3-catkin-tools \
        git \
        curl

RUN python3 -m pip install --upgrade odrive

RUN mkdir -p /root/ros_ws/src && \
    cd /root/ros_ws/src && \
    git clone https://github.com/YDLIDAR/ydlidar_ros && \
    cd ydlidar_ros && \
    git checkout master && \
    cd ../.. && \
    . /opt/ros/noetic/setup.sh && \
    catkin build

RUN curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - && \
    apt-get install -y nodejs

RUN python3 -m pip install \
    paho-mqtt

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt-get update && \
    apt-get install -y \
        ros-noetic-joy \
        ros-noetic-teleop-twist-joy \
        ros-noetic-twist-mux \
        ros-noetic-robot-localization \
        ros-noetic-gmapping \
        ros-noetic-move-base \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-compressed-image-transport \
        ros-noetic-realsense2-camera \
        ros-noetic-imu-filter-madgwick

COPY /web /root/web
RUN cd /root/web && \
    npm ci

COPY /ros/src /root/ros_ws/src
RUN . /opt/ros/noetic/setup.sh && \
    cd /root/ros_ws && \
    catkin build

COPY /scripts/start_basic.bash /
RUN chmod +x /start_basic.bash
COPY /scripts/start_full.bash /
RUN chmod +x /start_full.bash

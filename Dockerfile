FROM ros:noetic

RUN apt-get update && \
    apt-get install -y \
        libusb-dev \
        python3-pip \
        git \
        curl

RUN python3 -m pip install odrive

RUN mkdir -p /root/ros_ws/src && \
    cd /root/ros_ws/src && \
    git clone https://github.com/YDLIDAR/ydlidar_ros && \
    cd ydlidar_ros && \
    git checkout master && \
    cd ../.. && \
    . /opt/ros/noetic/setup.sh && \
    catkin_make

RUN curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - && \
    apt-get install -y nodejs

RUN apt-get update && \
    apt-get install -y \
        ros-noetic-joy \
        ros-noetic-teleop-twist-joy \
        ros-noetic-twist-mux \
        ros-noetic-robot-localization \
        ros-noetic-gmapping \
        ros-noetic-move-base

COPY /web /root/web
RUN cd /root/web && \
    npm init -y && \
    npm install express \
                socket.io \
                rosnodejs \
                arraybuffer-to-string

COPY /ros/src /root/ros_ws/src
WORKDIR /root/ros_ws
RUN . /opt/ros/noetic/setup.sh && \
    catkin_make

RUN python3 -m pip install paho-mqtt

WORKDIR /
COPY /entrypoint.bash /
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]

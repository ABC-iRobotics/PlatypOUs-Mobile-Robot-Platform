FROM ros:noetic

RUN apt-get update && apt-get install -y --no-install-recommends ros-noetic-desktop-full && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y \
        libusb-dev \
        libusb-1.0-0-dev \
        python3-pip \
        python3-catkin-tools \
        git \
        curl \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        mesa-utils \
    && \
    rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://deb.nodesource.com/setup_lts.x | bash - && \
    apt-get install -y nodejs

RUN python3 -m pip install --upgrade \
        pip \
        odrive \
        paho-mqtt

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
        ros-noetic-imu-filter-madgwick \
        ros-noetic-xacro \
        ros-noetic-robot-state-publisher \
        ros-noetic-tf-conversions \
        ros-noetic-global-planner \
        ros-noetic-dwa-local-planner \
        ros-noetic-effort-controllers \
        ros-noetic-velocity-controllers \
        ros-noetic-rtabmap-ros \
        ros-noetic-map-server \
        ros-noetic-rosbridge-suite

COPY /docker/entrypoint.bash /

ENTRYPOINT ["/entrypoint.bash"]





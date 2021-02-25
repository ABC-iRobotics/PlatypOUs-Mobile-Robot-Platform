docker run \
    -it \
    --privileged \
    -v /dev:/dev \
    platypous:latest roslaunch platy_bringup basic.launch

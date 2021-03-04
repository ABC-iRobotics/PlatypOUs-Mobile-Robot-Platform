docker run \
    -it \
    --rm \
    --privileged \
    -v /dev:/dev \
    --network host \
    platypous:latest roslaunch platy_bringup basic.launch

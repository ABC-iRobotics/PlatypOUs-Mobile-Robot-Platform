docker run \
    -it \
    --rm \
    --privileged \
    -v /dev:/dev \
    --network host \
    platypous:latest roslaunch platypous_launch basic.launch

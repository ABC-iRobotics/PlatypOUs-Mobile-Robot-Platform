#!/bin/bash

docker run \
    -i \
    --rm \
    --privileged \
    -v /dev:/dev \
    --network host \
    --name platypous_container \
    platypous:latest \
    roslaunch platypous_launch basic.launch &
sleep 5 && \
docker exec -it platypous_container \
    bash /entrypoint.bash node /root/web/index.js

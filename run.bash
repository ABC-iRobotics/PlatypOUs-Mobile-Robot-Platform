#!/bin/bash

docker run \
    -i \
    --rm \
    --privileged \
    -v /dev:/dev \
    --network host \
    --name platypous_container \
    platypous:latest \
    bash /start_script.bash

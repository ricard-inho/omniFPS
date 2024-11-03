#!/bin/bash
xhost +
docker run --name ros2-humble-zeroglab-container -it --rm --network=host --ipc=host ros2-humble-zeroglab:latest
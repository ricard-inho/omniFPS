# Floating Platform Lab Isaacsim ROS2 Humble

# Run with ROS2 Humble

Start simulation's docker

> [!IMPORTANT]
> Build docker first by running `./docker.fpl/build.sh`

```
./docker.fpl/run.sh
/isaac-sim/python.sh run.py
```


On another terminal start the ROS2 humble docker.

> [!IMPORTANT]
> Build docker first by running `./docker.ros2_humble/build.sh`

```
./docker.ros2_humble/run.sh
ros2 run ...
```
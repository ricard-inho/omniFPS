# Floating Platform Lab Isaacsim ROS2 Humble

# Run with ROS2 Humble

Start simulation's docker

> [!IMPORTANT]
> Build docker first by running `./docker.zeroGlab/build.sh`

```
./docker.zeroGlab/run.sh
/isaac-sim/python.sh run.py
```


On another terminal start the ROS2 humble docker.

> [!IMPORTANT]
> Build docker first by running `./docker.ros2_humble/build.sh`

```
./docker.ros2_humble/run.sh
ros2 topic pub --once /ZeroGLab/Robots/SpawnFP geometry_msgs/msg/PoseStamped "{'header':{'stamp':{'sec':0.0,'nanosec':0.0},'frame_id':'FloatingPlatform'},pose:{'position':{'x':'10.0','y':10.0,'z': 1.0},'orientation':{'x':0.0,'y':0.0,'z':0.0,'w':1.0}}}"
```
![OMNIFPS_LOGO](data/omnifps.png) 
Omniverse Floating Platform Simulator

## Installation

> [!IMPORTANT]
> Generate your NGC [key](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key).
> More [info](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim).

```
docker login nvcr.io
Username: $oauthtoken
Password: <Your Key>
docker pull nvcr.io/nvidia/isaac-sim:4.2.0
./docker.zeroGlab/build.sh
```


## Getting Started
```
./docker.zeroGlab/run.sh
/isaac-sim/python.sh run.py
```


On another terminal start the ROS2 humble docker.

> [!TIP]
> Build the docker first by running `./docker.ros2_humble/build.sh`

```
./docker.ros2_humble/run.sh
ros2 topic pub /AirBearingsSwitch std_msgs/Bool "data: true" -1
```

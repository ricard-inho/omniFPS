# Pingu MuJoCo Simulation

## Installation

Install `mujoco_ros2_control`.

Don't forget to install MuJoCo and setup environment variable.

```bash
export MUJOCO_DIR=/opt/mujoco-x.x.x # change it to your path
```

```bash
mkdir mjc_ws/src -p
cd mjc_ws/src
git clone git@github.com:aky-u/mujoco_ros2_control.git
```

Clone the repository.

```bash
cd
git clone git@github.com:ricard-inho/omniFPS.git
```

Link this folder under the workspace.

```bash
cd mjc_ws/src
ln -s ~/omniFPS/Mujoco/
```

Build and launch example.

```bash
cd ../ # mv to ws
colcon build --symlink-install
```

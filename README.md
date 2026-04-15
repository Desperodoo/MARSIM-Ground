# MARSIM
MARSIM: A light-weight point-realistic simulator for LiDAR-based UAVs

Paper is available on Arxiv: https://arxiv.org/abs/2211.10716

The video is available on youtube: https://youtu.be/hiRtcq-5lN0

## Ground Docker Quick Start

This branch includes a ground-robot adaptation of the MARSIM + FUEL stack:

- 3 Unitree-style quadruped visualizations in RViz
- ground-constrained multi-robot exploration launch
- fixed-trajectory forest coverage showcase launch
- rolling accumulated point cloud visualization for each robot
- simple visual gait animation for the quadrupeds in the fixed-trajectory showcase

## Common Commands

### 1. Clone and build

Recommended reproduction path on a fresh Ubuntu 20.04 machine:

```bash
git clone git@github.com:Desperodoo/MARSIM-Ground.git
cd MARSIM-Ground

# Optional if you need the same proxy route we used during development
export HTTP_PROXY=http://172.18.196.129:7890
export HTTPS_PROXY=$HTTP_PROXY
export ALL_PROXY=$HTTP_PROXY
export NO_PROXY=localhost,127.0.0.1,::1

./docker/build_all.sh
```

If you only need to rebuild the Docker catkin workspace after code changes:

```bash
./docker/build_workspace.sh
```

If your user is not in the `docker` group, the scripts will automatically fall back to `sudo docker`.

### 2. X11 / RViz preparation

If you want RViz from inside Docker on a local X11 desktop:

```bash
xhost +local:root
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
```

### 3. Run examples

Run the fixed-trajectory forest coverage showcase:

```bash
./docker/run_showcase.sh
```

Run the multi-robot exploration version:

```bash
./docker/run_exploration_multi.sh
```

Useful toggles:

```bash
ENABLE_RVIZ=false ./docker/run_showcase.sh
USE_GPU=true ./docker/run_showcase.sh
AUTOSTART=false ./docker/run_exploration_multi.sh
MAP_NAME=$(pwd)/map_generator/resource/small_forest01cutoff.pcd ./docker/run_showcase.sh
```

The fixed-trajectory showcase now starts each quadruped at the beginning of its assigned route, so the robot no longer visually jumps from a separate spawn pose to the path start.

### 4. Record RViz

Record only the visible RViz region from the host X11 desktop:

```bash
DURATION_SECONDS=180 ./docker/record_rviz_region.sh
```

This helper locates the `multi_ground.rviz - RViz` window and records that screen region to `recordings/`. It is a host-side region capture, not true window capture, so if another window covers RViz that overlap will also appear in the video.

Record the RViz X11 window directly so overlapping windows such as VS Code do not appear in the output:

```bash
DURATION_SECONDS=180 ./docker/record_rviz_window.sh
```

If you want one command that launches the fixed-trajectory showcase and immediately starts an RViz-only recording:

```bash
DURATION_SECONDS=180 ./docker/run_showcase_record_window.sh
```

If the auto-record script says another showcase instance is already running, clear the old one first:

```bash
pkill -f trajectory_showcase_ground.launch || true
```

### 5. Notes

- `./docker/run_showcase.sh` and `./docker/run_exploration_multi.sh` auto-detect whether Docker GPU passthrough is available. If an NVIDIA runtime is available, GPU rendering is enabled automatically; otherwise they fall back to the non-GPU path.
- The fixed-trajectory ground showcase includes a lightweight visual gait effect for the Unitree-style robots so they no longer appear to slide rigidly through the scene.
- The Docker workflow uses a local catkin workspace at `.docker_ws/` and mounts this repository into the container as `/root/marsim_ws/src/MARSIM_fuel`.

## Update

Ubuntu 20.04 is also supported in ubuntu20 branch.

**Ten realistic maps (low and high resolution) have been realeased in the realease packages.**

**A new branch that merge with FUEL has been released in the fuel_ubuntu20 branch.**

## Prerequisited

### Ubuntu and ROS

Ubuntu 16.04~20.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### PCL && Eigen && glfw3

PCL>=1.6, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

glfw3:
```
sudo apt-get install libglfw3-dev libglew-dev
```

### make
```
mkdir -p marsim_ws/src
cd marsim_ws/src
git clone git@github.com:hku-mars/MARSIM.git
cd ..
catkin_make
```

## run the simulation

```
source devel/setup.bash
roslaunch test_interface single_drone_avia.launch
```
Click on 3Dgoal tool on the Rviz, you can give the UAV a position command to control its flight.

For now, we provide several launch files for users, which can be found in test_interface/launch folder.

You can change the parameter in launch files to change the map and LiDAR to be simulated.

** If you want to use the GPU version of MARSIM, please set the parameter "use_gpu" to true. **

## run the simulation with FUEL algorithm

You should first change the branch to fuel_ubuntu20 branch. If you are using ubuntu 20.04, you should first download Nlopt and make install it in your environment. Then you can run the simulation by the command below:
```
source devel/setup.bash
roslaunch exploration_manager exploration.launch
```
Then click on 2Dgoal tool on the Rviz, randomly click on the map, and FUEL would automously run.

## Acknowledgments
Thanks for [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL.git)

## Future
More realistic maps and functions are going to be released soon.

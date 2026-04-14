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

If you want RViz from inside Docker on a local X11 desktop:

```bash
xhost +local:root
export DISPLAY=${DISPLAY:-:0}
export XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}
```

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
```

The Docker workflow uses a local catkin workspace at `.docker_ws/` and mounts this repository into the container as `/root/marsim_ws/src/MARSIM_fuel`.

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

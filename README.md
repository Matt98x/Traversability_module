# Traversability_module

## Prerequisites

Ubuntu bionic 18.04.5
ROS Melodic
ROS Mesh tool

## Build 
Copy the package from the repository and put it inside the src folder of the workspace.



Instruction:
On the shell type 'gedit ~/.bashrc', when opened copy the following text at the end of it, modifying the "PATH_WORKSPACE" to the correct address:
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/velodyne_plugin/build
source /opt/ros/melodic/setup.bash
source /usr/share/gazebo/setup.sh
export PATH_WORKSPACE=~/Desktop/Thesis_workspace/
export PATH_MAIN=${PATH_WORKSPACE}/src/Traversability_module/Robot
source ${PATH_WORKSPACE}/devel/setup.bash
export ROS_PACKAGE_PATH=${PATH_MAIN}/unitree_ros:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=${PATH_WORKSPACE}/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${PATH_WORKSPACE}/devel/lib:${LD_LIBRARY_PATH}
export UNITREE_LEGGED_SDK_PATH=${PATH_MAIN}/unitree_legged_sdk
export ALIENGO_SDK_PATH=${PATH_MAIN}/aliengo_sdk
#amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
```
Correct the various mistakes till the code is running correctly, some commands that have corrected the behavior are:

```
apt install libprotobuf-dev
sudo dpkg --configure -a
sudo apt-get -f install
```

May have problem with the version of gedit, if it happens, remove it with:
```
sudo apt-get purge gedit
```


Installation of the controllers on melodic:
```
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```

Compile the code:
```
cd "Workspace path"
catkin_make
```

To run a first visualization on RVIZ:
```
roslaunch a1_description a1_rviz.launch
```

To launch the gazebo template environment:
```
roslaunch unitree_gazebo normal.launch rname:=a1 wname:=stairs
```

Install this (REQUIRED)
```
sudo apt install ros-melodic-velodyne-gazebo-plugins
```





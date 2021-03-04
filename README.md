# Traversability_module

## Prerequisites

Ubuntu bionic 18.04.5
ROS Melodic
ROS Mesh tool

## Build 
Copy the package from the repository and put it inside the src folder of the workspace.

Installation of the controllers on melodic:
```
sudo apt-get install ros-melodic-controller-interface  ros-melodic-gazebo-ros-control ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-joint-trajectory-controller
```

Install libprotobuf-dev to correct the google dependency problem.
```
apt install libprotobuf-dev
```


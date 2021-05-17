#!/bin/bash

cd Third_parties/Mesh/lvr_ros/src
rm reconstruction.cpp conversions.cpp
roscd traversability_mesh_pkg
cd src/Modified_lvr_code
cp reconstruction.cpp ~/Desktop/catkin_ws/src/Traversability_moduleThird_parties/Mesh/lvr_ros/src
cp conversions.cpp ~/Desktop/catkin_ws/src/Traversability_moduleThird_parties/Mesh/lvr_ros/src

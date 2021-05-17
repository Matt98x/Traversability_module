#!/bin/bash

cd ~/Desktop/catkin_ws/src/Traversability_module/Third_parties/Mesh/lvr_ros/src
cd Third_parties/Mesh/lvr_ros/src
rm reconstruction.cpp conversions.cpp
cd ~/Desktop/catkin_ws/src/Traversability_module/traversability_mesh_package/src/Modified_lvr_code
cp reconstruction.cpp ~/Desktop/catkin_ws/src/Traversability_module/Third_parties/Mesh/lvr_ros/src/reconstruction.cpp
cp conversions.cpp ~/Desktop/catkin_ws/src/Traversability_module/Third_parties/Mesh/lvr_ros/src/conversions.cpp
echo >> ~/Desktop/catkin_ws/src/Traversability_module/Third_parties/Mesh/lvr_ros/src/reconstruction.cpp
sed -i '$ d' ~/Desktop/catkin_ws/src/Traversability_module/Third_parties/Mesh/lvr_ros/src/reconstruction.cpp
echo >> ~/Desktop/catkin_ws/src/Traversability_module/Third_parties/Mesh/lvr_ros/src/conversions.cpp
sed -i '$ d' ~/Desktop/catkin_ws/src/Traversability_module/Third_parties/Mesh/lvr_ros/src/conversions.cpp
cd ~/Desktop/catkin_ws
catkin_make_isolated

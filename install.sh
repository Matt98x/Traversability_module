#!/bin/bash

# Variable declaration
package_path=$(pwd)
path="${package_path%%/src*}"

# Environment setup for the first installation
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/velodyne_plugin/build
source /opt/ros/melodic/setup.bash
source /usr/share/gazebo/setup.sh
export PATH_WORKSPACE=${path}
export PATH_MAIN=$package_path'/Third_parties/Robot'
source ${PATH_WORKSPACE}/devel_isolated/setup.bash
export ROS_PACKAGE_PATH=${PATH_MAIN}/unitree_ros:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=${PATH_WORKSPACE}/devel_isolated/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=${PATH_WORKSPACE}/devel/lib:${LD_LIBRARY_PATH}
export UNITREE_LEGGED_SDK_PATH=${PATH_MAIN}/unitree_legged_sdk
export ALIENGO_SDK_PATH=${PATH_MAIN}/aliengo_sdk
#amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"

# Inport the same command in the bashrc file so that opening a new terminal will automatically setup the environment
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/velodyne_plugin/build' >> ~/.bashrc
echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export PATH_WORKSPACE='${path} >> ~/.bashrc
echo 'export PATH_MAIN='$package_path'/Third_parties/Robot' >> ~/.bashrc
echo 'source ${PATH_WORKSPACE}/devel_isolated/setup.bash' >> ~/.bashrc
echo 'export ROS_PACKAGE_PATH=${PATH_MAIN}/unitree_ros:${ROS_PACKAGE_PATH}' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=${PATH_WORKSPACE}/devel_isolated/lib:${GAZEBO_PLUGIN_PATH}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=${PATH_WORKSPACE}/devel/lib:${LD_LIBRARY_PATH}' >> ~/.bashrc
echo 'export UNITREE_LEGGED_SDK_PATH=${PATH_MAIN}/unitree_legged_sdk' >> ~/.bashrc
echo 'export ALIENGO_SDK_PATH=${PATH_MAIN}/aliengo_sdk' >> ~/.bashrc
echo '#amd64, arm32, arm64' >> ~/.bashrc
echo 'export UNITREE_PLATFORM="amd64"' >> ~/.bashrc

# Change to the catkin workspace to build the package
cd $path
catkin_make_isolated

# Rosdep loop through every package in the repository to install the dependencies
rosdep install a1_description
rosdep install aliengo_description
rosdep install hdf5_map_io
rosdep install laikago_description
rosdep install laserscan_to_pointcloud
rosdep install mesh_msgs
rosdep install label_manager
rosdep install lvr_ros
rosdep install mesh_msgs_conversions
rosdep install mesh_msgs_hdf5
rosdep install mesh_msgs_transform
rosdep install mesh_tools
rosdep install pcl
rosdep install mesh_to_pointcloud
rosdep install pcl_msgs
rosdep install pcl_conversions
rosdep install pcl_ros
rosdep install perception_pcl
rosdep install pose_to_tf_publisher
rosdep install dynamic_robot_localization
rosdep install rviz_map_plugin
rosdep install rviz_mesh_plugin
rosdep install traversability_mesh_package
rosdep install unitree_legged_msgs
rosdep install unitree_controller
rosdep install unitree_gazebo
rosdep install unitree_legged_control
rosdep install unitree_legged_real


# Repeat the catkin make to ensure the installation
catkin_make_isolated

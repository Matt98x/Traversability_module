cmake_minimum_required(VERSION 3.0.2)
project(traversability_mesh_package)

set(PACKAGE_DEPENDENCIES
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  genmsg
  mesh_msgs
  message_generation
  message_runtime
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  label_manager
  tf2_ros
  lvr_ros
  cv_bridge
  image_transport
)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(LVR2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${LVR2_DEFINITIONS} ${OpenCV_DEFINITIONS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


add_message_files(
  FILES
  Features.msg
  Base.msg
	Data.msg
	GeoFeature.msg
	Proximals.msg
	RobotState.msg
  AccMetrics.msg
  Traversability.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  mesh_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)




################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS LVR2 MPI
)

###########
## Build ##
###########

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOTHER_STUFF=1" )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LVR2_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${lvr_ros_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/geometry_score.cpp 
    src/visual_score.cpp
    src/precision_score.cpp
    src/myNeuralNetworkFunction.cpp
    src/myNeuralNetworkFunction_data.cpp
    src/myNeuralNetworkFunction_initialize.cpp
    src/rt_nonfinite.cpp
    include/tmwtypes.h
    include/rtwtypes.h
    include/myNeuralNetworkFunction_types.h
)
## EXECUTABLES

## cloud handler executable
add_executable(cloud_handler src/cloud_handler.cpp)
target_link_libraries(cloud_handler
  ${catkin_LIBRARIES}
  ${LVR2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(cloud_handler ${catkin_EXPORTED_TARGETS})


## Flow regulator executable
add_executable(flow_regulator src/flow_regulator.cpp)
target_link_libraries(flow_regulator
  ${catkin_LIBRARIES}
  ${LVR2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(flow_regulator ${catkin_EXPORTED_TARGETS})

## Geometry features extractor executable
add_executable(geometry_extractor src/geometry_extractor.cpp)
target_link_libraries(geometry_extractor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)
add_dependencies(geometry_extractor ${catkin_EXPORTED_TARGETS})

## Visual features extractor executable
add_executable(visual_extractor src/visual_extractor.cpp)
target_link_libraries(visual_extractor
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)
add_dependencies(visual_extractor ${catkin_EXPORTED_TARGETS})

## Pointcloud precision extractor executable
add_executable(point_extractor src/point_extractor.cpp)
target_link_libraries(point_extractor
  ${catkin_LIBRARIES}
  ${LVR2_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(point_extractor ${catkin_EXPORTED_TARGETS})

## Traversability computer executable
add_executable(traversability_score src/traversability_score.cpp)
target_link_libraries(traversability_score
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
	${PROJECT_NAME}
)
add_dependencies(traversability_score ${catkin_EXPORTED_TARGETS})

## Robot State publisher
add_executable(robot_state src/robot_state.cpp)
target_link_libraries(robot_state
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)
add_dependencies(robot_state ${catkin_EXPORTED_TARGETS})

## Mesh display
add_executable(Traversability_display src/Traversability_display.cpp)
target_link_libraries(Traversability_display
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)
add_dependencies(Traversability_display ${catkin_EXPORTED_TARGETS})

add_executable(rmdt src/rmdt.cpp)
target_link_libraries(rmdt
  ${catkin_LIBRARIES}
)
add_dependencies(rmdt ${catkin_EXPORTED_TARGETS})

## Alternative State publisher for robot application
add_executable(alternative_state src/alternative_state.cpp)
target_link_libraries(alternative_state
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)
add_dependencies(alternative_state ${catkin_EXPORTED_TARGETS})

## Alternative State publisher for robot application
add_executable(relayer1 src/relayer1.cpp)
target_link_libraries(relayer1
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)
add_dependencies(relayer1 ${catkin_EXPORTED_TARGETS})

## Alternative State publisher for robot application
add_executable(relayer2 src/relayer2.cpp)
target_link_libraries(relayer2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)
add_dependencies(relayer2 ${catkin_EXPORTED_TARGETS})

## Alternative State publisher for robot application
add_executable(synchronizer src/synchronizer.cpp)
target_link_libraries(synchronizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)
add_dependencies(synchronizer ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_traversability_mesh_package.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

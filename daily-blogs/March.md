## To-dos
- Re-organize the Readme.md
- Write a bash or a docker image script to setup the repository when installed on another computer
- Write a script to assign traversability on just the geometric properties of the mesh elements
- Find a way to display the mesh on RViz (some form of SLAM might be required)
- Test whether the robot movement deform the pointcloud (robot control is required)
- Write an intermediate script for point cloud parameters adjustment(i.e. publication rate with respect to update rate) -> this is done to avoid big measurements errors by the LiDAR while the robot is moving, while still performing the mesh reconstraction at an appropriate rate
- Correctly configure Dynamic robot localization

## March 22(Monday)
- Package reorganization
- Attempts to configure the Dynamic robot localization 
- Correction to the script Mesh constructor and to its name (Now Flow_generator)

## March 21(Sunday)
- Localization problem solved in three ways depending on the environment:
   - Simple gazebo provided odometry (produced with the c++ node unitree_transform) and thought to simplify the simulation
   - Dynamic\_robot\_localization package installed and thought for general case localization, even if not yet correctly configured (there are some confusions on how it works) 
   - Studied Markers-based localizations algorithms

## March 20(Saturday)
- Research for new solutions for both problems

## March 19(Friday)
- Abandonment of free-gait as the control package since a needed component of the package(the application to the robot) is not open-source and no solutions have been found to correct it
- Abandonment of RTab-ros and general SLAM algorithms as solution for localization

## March 18(Thursday)
- Introduction of free-gait as the control package (Yet to be properly included)
- Introduction of RTab-ros as SLAM algorithm (Yet to be properly included)

## March 14 (Sunday)
- Work on the presentation og the 17th of March
- Update of the To-dos list

## March 13 (Saturday)
- Development of some architecture details
- Work on the presentation of the 17th of March
- Update of the To-dos list

## March 12 (Friday)
- Mesh creation is working and the reconstruction is done directly on the published Point Cloud
- Allow the openCl platform to work on Intel processor instead of just GPUs
- Mesh constructor is temporarily unused as a different working approach was found. Still, it will be recovered to allow for the control of the publication of the Point Cloud at a different rate than what published by the LiDAR (renamed "flow regulator"). 

## March 11 (Thursday)
- Produce the mesh constructor
  - Write the c++ script to read the PointCloud and publish the mesh
  - Modify the CMakeList 
  - Modify the Package.xml 

## March 10 (Wednesday)



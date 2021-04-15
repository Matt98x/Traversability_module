## To-dos
- Re-organize the Readme.md
- Write a bash script to setup the repository when installed on another computer
- Write a script to assign traversability on just the geometric properties of the mesh elements
- Correctly configure Dynamic robot localization
- Create a simple trunk pose controller for the simulation environment (user-controlled)

## April 15 (Thursdaz)
- Finally installed the package on the the computer assigned to me:
	- I had to temporarily get rid of the dynamic robot localization package since it helped speeding up the installation 
	- I got rid of the unitree\_legged\_real, since it needed a working lcm-1.4.0 package which I have yet to fix, but since i'm not interested in the robot control is actually fine
	- The code is finally running with decent performance, 27 fps in the rviz simulation against the 1fps of the virtual machine on my pc
	- The problem that arouse is the wrong measurements I've been taking of the lvr-ros reconstruction performance: I thought the computational time for one element would be around 100 ms because I was taking the computational time as being the input rate in ms plus the time the node took to compute the mesh. Instead I find myself with a total computational rate for the node of 0.54 Hz. The ripercussions of this fact are not really a ptroblem if the robot speed is relatively low(for example 0.5 m/s) but could become really complicated if the movement speed was higher or if the angular deviation inbetween mesh computation was over a certain threshold. For the most part there are no real solutions, but to either adapt the robot movement to the mesh reconstruction contraints or to change the reconstruction algorithm to be more efficient(reducing the pointcloud density or the field of view). Both of these possibilities have been studied but I haven't decided on which implementing since I would like to know the application conditions first.

## April 14 (Wednesday)
- Esperience in the lab to understand the robot and its control:
	- Practiced with the robot control, moving it around, switching between control modes and trying to memorize the commands
	- Analyzed the architectural structure of the robot with its three computers:
		- Raspberry Pi: with its interfaces (Hdmi and 2 usb ports) in the frontal part of the robot(near the camera), is a soft real time computer running Ubuntu 16.04 without a ROS environment. It is directly connected to the Robot controller via ethernet connection and will be the one used to receive the motion controls from the user
		- Nvidia TX2: is a non real time computer placed in the rear part of the robot that handles the optional capabilities of the robot(SLAM algorithm and interface with the APP). This system is running Ubuntu bionic 18.04.4 and might be use to run the package or to extract the data to be used in the package(PointCloud and image) via UDP or TCP connection with an external computer.
		- Robot control: it's a hard real-time system which seem to be inaccessible to the user and takes commands via ethernet from the Raspberry Pi. Has two addresses, one for the Normal control and one for the sport control, handling the movement via two types of messagges: low-level and high level. The first is mainly focused in controlling the motor toque, velocity and position. The second focuses on controlling on a higher level the speed of the robot, its orientation, the step high and so on.
- Research into the use of UDP or TCP transmission of PointClouds and images:
	- Found some examples that use the kinect to do it, with a python code at this [link](https://forum.processing.org/two/discussion/11090/kinect-depth-data-via-udp), with the problems of trying to substitute the kinect with the realsense device while understanding how the compression used in the code might influence the transmission and the performance of my code.
	- I've yet to find some code to extract the data from the robot sensors, since the suspicion is that I could have them through one of the many ethernet connection of the TX2 system, but it is yet to be proved.
	- If the code is to be runned externally to the robot some portable device is to be used when testing outdoor. Tests must be performed on my computer to see whether it can perform this task.
- Modification of the repository structure to have submodules:
	- All the packages in the "Third_parties/Mesh" folder have been converted in fully functional submodules, with the .submodules accounting for the correct URL and path inside my repository
	- The dynamic robot localization packages have been temporarily deleted to improve installation and building time expecially when for now they are not utilized. When they will be useful again I will install them as submodules
- Research into monkey patching for the lvr-ros script (conversion.cpp): no public solution have been found, for now the phylosofy will be, in the installation script, to sobstitute the script inside the folder with one saved in the traversability\_mesh\_package with the one present in lvr_ros, and every time the submodule is updated running the installation script. This solution will work momentarily, until there won't be an error running the catkin\_make with the updated lvr-ros package. 
 
## April 13 (Tuesday)
- Updated Professor Solari on the state of the project and rapidly read the paper he gave me to compute the rugosity of the terrain in relation to the number of points I use (this problem arise from the fact that the density of the poincloud is related to the distance from the robot), unfortunately the method cannot be easily applied to my case. A possible solution might be to multiply the rugosity by some coefficient related to the real area of the mesh element and the certainty related to the distance from the robot.
- Introduced the synchronizer inside the flow_regulator
- Extracted the transformation between the Lidar and the robot torso to enable the extraction of the global position of the points in the PointCloud; Which will be useful when we will study the traversability
- I extracted a way of determining the most useful normal to each mesh element; The main concept is that the dot product of the normal and the vector connecting the Lidar origin and the element baricenter should have negative sign 
- I think I will add another synchronizer for the processing nodes since I have to impose a strong condition on working on data all related to the same time slice, this complicate a bit the computation of the worst computational time since every process is related. To solve this, we can compute the difference between the time at which the output is published and the timestamp of the message itself
- I'm exploring the possibilty of using multi-threading for all computations inside the feature extraction nodes and the traversability score calculator as to improve the required computational time.

## April 12 (Monday)
- Retrieved the student pass so that on Wednesday I can go to the lab
- After yesterday modifications to enable the installation in other computers, I tried to change the nature of the submodules inside the package. Unfortunately, this led to the corruption of the repository, which modified expecially the rviz\_mesh\_plugin package. In fact, after I set the repository back to a previous commit, the package disappeared and I could not retrieve it. After downloading a package with the same name and same nature, and after building and sourcing, this message appeared:
```
[ERROR] [1618260240.824223000, 20.986000000]: PluginlibFactory: The plugin for class 'rviz_mesh_plugin/TriangleMesh' failed to load.  Error: According to the loaded plugin descriptions the class rviz_mesh_plugin/TriangleMesh with base class type rviz::Display does not exist. Declared types are  octomap_rviz_plugin/ColorOccupancyGrid octomap_rviz_plugin/OccupancyGrid octomap_rviz_plugin/OccupancyGridStamped octomap_rviz_plugin/OccupancyMap octomap_rviz_plugin/OccupancyMapStamped rtabmap_ros/Info rtabmap_ros/MapCloud rtabmap_ros/MapGraph rviz/Axes rviz/Camera rviz/DepthCloud rviz/Effort rviz/FluidPressure rviz/Grid rviz/GridCells rviz/Illuminance rviz/Image rviz/InteractiveMarkers rviz/LaserScan rviz/Map rviz/Marker rviz/MarkerArray rviz/Odometry rviz/Path rviz/PointCloud rviz/PointCloud2 rviz/PointStamped rviz/Polygon rviz/Pose rviz/PoseArray rviz/PoseWithCovariance rviz/Range rviz/RelativeHumidity rviz/RobotModel rviz/TF rviz/Temperature rviz/WrenchStamped rviz_map_plugin/ClusterLabel rviz_map_plugin/Map3D rviz_map_plugin/Mesh rviz_plugin_tutorials/Imu
```  
Which, in a tutorial on ros.org was related to some mistake in the mmanifest(synonim for package.xml), but in this case, this problem was not found.  
Further investigations revealed that the package, although in the catkin list, was not in the package stack, which is strange because I can do a catkin\_make\_isolated, but not a roscd or a rospack.  
I then tried to do a catkin\_make and sourcing the devel setup script, finally obtaining the plugin on rviz, but when I tried launching the package this error appeared:
```
[ERROR] [1618259896.975292700, 21.726000000]: PluginlibFactory: The plugin for class 'rviz_mesh_plugin/TriangleMesh' failed to load.  Error: Could not find library corresponding to plugin rviz_mesh_plugin/TriangleMesh. Make sure the plugin description XML file has the correct name of the library and that the library actually exists.
```  
Furthermore, I could not launch the scripts of the package.  
After another research session I found out why the package have disappeared: In the most updated version of the mesh tool, the package have been deleted.  
This leaves me to choose between two strategies:  
	- Use the last version of the tool and re-adapt how I display the mesh, which would imply having all the advantages of the most recent version at the cost of having to update my display strategy every time the package is modified  
	- Look for the version I have used till now and never update the package again, which would allow to work with something I have already produced and even introduce changes of my own, at the cost of all the problem this version might have

Since I can't seem to find the version, for now I will modify my work to use the latest version.  
Now the display use the message mesh_msgs/MeshGeometryStamped and reads directly the reconstruction output.

=======
## April 11 (Sunday)
- I produced the bash installation script: the script automatically set the environment compile the code and should be able to install all the libraries. Unfortunately, there are some problems in the installation:
	- Some packages are not downloaded properly through the git process, the folders are in place but they are empty. I'm working on it, trying to modifying the packages from being submodules, I think this is the problem. 
	- I could not use rosdep feature because it has not been initialized on the computer and I tried to initialize it using the super user credentials( I thought they were active and the passworld were the same as my Gaspar credentials)
	- I found out that there is a package I have not considered(lvr2), which I will add to the structure as an auxiliary(I will try to install in a way that will be correctly installed in other computers)
  
At any rate, from the trials I made on my computer all commands are correcly executed and working, but the environment is not compliant, In the trials I made the rosdep install does not require the sudo command while the rosdep init seems to require it.

regarding the bug I talked about on the 9th, here's the transcription:  
```
lvr_ros_reconstruction: /usr/include/boost/smart_ptr/shared_array.hpp:199: T& boost::shared_array<T>::operator[](std::ptrdiff_t) const [with T = float; std::ptrdiff_t = long int]: Assertion `px != 0' failed.
/root/Desktop/Thesis_workspace/devel_isolated/lvr_ros/lib/lvr_ros/lvr_ros_reconstruction: line 1:  8210 Aborted                 $0 $@
\[lvr-11\] process has died \[pid 7882, exit code 134, cmd bash -c sleep 50; $0 $@ /root/Desktop/Thesis_workspace/devel_isolated/lvr_ros/lib/lvr_ros/lvr_ros_reconstruction -d /root/Desktop/Thesis_workspace/src/Traversability_module/Third_parties/Robot/unitree_ros/unitree_gazebo/rviz/interface.rviz __name:=lvr __log:=/root/.ros/log/54e5e20a-9add-11eb-8d6f-0242ac110002/lvr-11.log\].
log file: /root/.ros/log/54e5e20a-9add-11eb-8d6f-0242ac110002/lvr-11*.log  
```
Again, a temporary solution have been found commenting the portion related to texture and color, in the function at line 90 of the script conversion.cpp (inside the src folder of the lvr\_ros package,part of the third\_parties folder). This allows to avoid the use of the boost library by avoiding the function call, in later work I will pin-point the function in which I use it, once I have it I can try to correct the problem or mantain as much code as I can 

regarding the bug I talked about on the 9th, here's the transcription:
'''
lvr_ros_reconstruction: /usr/include/boost/smart_ptr/shared_array.hpp:199: T& boost::shared_array<T>::operator[](std::ptrdiff_t) const [with T = float; std::ptrdiff_t = long int]: Assertion `px != 0' failed.
/root/Desktop/Thesis_workspace/devel_isolated/lvr_ros/lib/lvr_ros/lvr_ros_reconstruction: line 1:  8210 Aborted                 $0 $@
\[lvr-11\] process has died \[pid 7882, exit code 134, cmd bash -c sleep 50; $0 $@ /root/Desktop/Thesis_workspace/devel_isolated/lvr_ros/lib/lvr_ros/lvr_ros_reconstruction -d /root/Desktop/Thesis_workspace/src/Traversability_module/Third_parties/Robot/unitree_ros/unitree_gazebo/rviz/interface.rviz __name:=lvr __log:=/root/.ros/log/54e5e20a-9add-11eb-8d6f-0242ac110002/lvr-11.log\].
log file: /root/.ros/log/54e5e20a-9add-11eb-8d6f-0242ac110002/lvr-11*.log
'''
Again, a temporary solution have been found commenting the portion related to texture and color, in the function at line 90 of the script conversion.cpp (inside the src folder of the lvr\_ros package,part of the third\_parties folder). This allows to avoid the use of the boost library by avoiding the function call, in later work I will pin-point the function in which I use it, once I have it I can try to correct the problem or mantain as much code as I can 

=======

## April 9 (Friday)
- A bug have appeared after the modifications I have done yesterday(most probably the correction I have done to solve it previously have been deleted by some error in merging and updating the repository): Inside the lvr_ros package an error occur where the condition px!=0 is not satisfied, as far as the debug has explored the problem is caused by something in the boost library when the method tries to handle the color and textures components of the pointcloud, but the specifics are yet to be explored in depth...A temporary solution have been found commenting the portion related to texture and color, in the function at line 90 of the script conversion.cpp (inside the src folder of the lvr\_ros package,part of the third\_parties folder)
- Creation of the youtube channel and found a way to capture the screen(Using the share screen feature of Microsoft Teams): the problem with this solution is that the combined computational requirement on the CPU and RAM of handling the Windows operating system, the Ubuntu operating system and the mesh creation and display makes the simulation move at 1 fps with a simulated time factor of around 0.34 which is extremely difficult to use to show the real performance but can give an idea of the working principle
- Installation on the University computer of rosdep and creation of the catkin workspace where the package will run
- Search of tutorials and method to configure the robot and set up the connection:
	- Useful tutorials are yet to be found and for now there is just the creaction of the connection between the robot and an external computer via wi-fi
	- It was discovered that to produce the connection it is necessary to connect the robot to a keyboard and a monitor, which gives the following possibilities:
		- run the code on a fixed computer and simply obtaining the sensor data via wireless connection: the pros of this version is that a fixed computer does not imply the interaction with other parts of the robot control but may not work with the real performance as the communication delay may be not negligeable 
		- run the code on the robot itself and display the results on a fixed computer: which implies acting on the robot architecture itself(even if only by taking part of the computational resources) but is as close to the real performance as we can get
	- The control of the real robot can be done in two ways: with the controller or sending low or high level controls through the wireless connection. For my experiments and for the sake of reducing as much as I can the workload not finalized in the production of the package, the first method should be the best for the real robot while in the simulation there will be a position control that does not make the legs move but just decide the next position of the torso
- Optimization of the geometrical features extractor algorithm: Instead of making the trasformation of the vertices position (from the camera to the world reference system) happen separately from the mesh elements features extraction, for each element vertex a check will tell if they have been transformed or not. In the negative case, the position in the world frame will be computed thanks to the transformation matrix we get from the pose of the robot and saved in a sort of dictionary(storing the information that the vertex have been transformed). Otherwise, the information is already available and some computational time is saved. Moreover we memorize the neighbours of each element noting the triangles that shares at least a vertex with, and from that we can find points with considerable height difference(as the normal between the elements will be noticeably different)
 
=======

## April 8(Thursday) and previous week
- Addition to the test description, still looking to ground the experiments and look for other metrics
- Additional study of the geometrical features, now defined(for each mesh element as):
	- baricenter: defined as the mean of the element vertices position; this measure is useful to compute the distance of a mesh element from the robot or the step height
	- inclination/slop: how inclined the surface is; it can be found with acos(optimized inside the code to reduce computational power compromising a little precision) between the surface normal and the vertical versor.
	- orientation: defined as the versor orthogonal to the mesh surface
- Study of geometrical traversability metrices:
	- distance: of the mesh elements from the robot or from a designated foot(yet to be decided)
	- slope: as previously defined
	- step height: computed as a function of the distance between the line passing through the robot center and oriented as the robot itself and the horizontal line passing through the baricenter and orthogonal to the first
	- there are some ideas regarding the extraction of a cost related to the static stability of the robot where the closest foot be placed in there(an article have been found but the implementation scheme is still to be refined and developed)
	- other metrics may be added in the future
- Creation of a matlab script to compute the worst case scenario arrival time distance between topics when the time approximator policy is applied in the flow regulator: result are yet to be fully interpreted but seems fine for the required application precision even considering "high speed" 
- Research in the rosdep tool which will allow for the automatic installation of packages-required libraries without the need of super user credential
- Development of the installation algorithm with the ability to automatically write in the source code the correct paths: theoretically completely defined but yet to be implemented
- Modification to the Readme to correct the links format

## To-dos
- Re-organize the Readme.md
- Write a bash script to setup the repository when installed on another computer
- Write a script to assign traversability on just the geometric properties of the mesh elements
- Correctly configure Dynamic robot localization
- Create a simple trunk pose controller for the simulation environment (user-controlled)

## April 11 (Sunday)
- I produced the bash installation script: the script automatically set the environment compile the code and should be able to install all the libraries. Unfortunately, there are some problems in the installation:
	- Some packages are not downloaded properly through the git process, the folders are in place but they are empty. I'm working on it, trying to modifying the packages from being submodules, I think this is the problem. 
	- I could not use rosdep feature because it has not been initialized on the computer and I tried to initialize it using the super user credentials( I thought they were active and the passworld were the same as my Gaspar credentials)
	- I found out that there is a package I have not considered(lvr2), which I will add to the structure as an auxiliary(I will try to install in a way that will be correctly installed in other computers)
  
At any rate, from the trials I made on my computer all commands are correcly executed and working, but the environment is not compliant, In the trials I made the rosdep install does not require the sudo command while the rosdep init seems to require it.

regarding the bug I talked about on the 9th, here's the transcription:
'''
lvr_ros_reconstruction: /usr/include/boost/smart_ptr/shared_array.hpp:199: T& boost::shared_array<T>::operator[](std::ptrdiff_t) const [with T = float; std::ptrdiff_t = long int]: Assertion `px != 0' failed.
/root/Desktop/Thesis_workspace/devel_isolated/lvr_ros/lib/lvr_ros/lvr_ros_reconstruction: line 1:  8210 Aborted                 $0 $@
\[lvr-11\] process has died \[pid 7882, exit code 134, cmd bash -c sleep 50; $0 $@ /root/Desktop/Thesis_workspace/devel_isolated/lvr_ros/lib/lvr_ros/lvr_ros_reconstruction -d /root/Desktop/Thesis_workspace/src/Traversability_module/Third_parties/Robot/unitree_ros/unitree_gazebo/rviz/interface.rviz __name:=lvr __log:=/root/.ros/log/54e5e20a-9add-11eb-8d6f-0242ac110002/lvr-11.log\].
log file: /root/.ros/log/54e5e20a-9add-11eb-8d6f-0242ac110002/lvr-11*.log
'''
Again, a temporary solution have been found commenting the portion related to texture and color, in the function at line 90 of the script conversion.cpp (inside the src folder of the lvr\_ros package,part of the third\_parties folder). This allows to avoid the use of the boost library by avoiding the function call, in later work I will pin-point the function in which I use it, once I have it I can try to correct the problem or mantain as much code as I can 


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

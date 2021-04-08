## To-dos
- Re-organize the Readme.md
- Write a bash script to setup the repository when installed on another computer
- Write a script to assign traversability on just the geometric properties of the mesh elements
- Correctly configure Dynamic robot localization
- Create a simple trunk pose controller for the simulation environment (user-controlled)

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

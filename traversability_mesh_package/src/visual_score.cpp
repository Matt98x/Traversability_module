// Tf libraries
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
// Vector related libraries
#include <iostream>
#include <vector>
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort
// Libraries related to quaternions manipulation
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// Math
#include <math.h>
// Message libraries
#include <traversability_mesh_package/RobotState.h>
#include <traversability_mesh_package/GeoFeature.h>
// Score calculation library
#include <score_calculators.h>

using namespace std;


//* Geometry based traverversability score calculator
float vis_calculator(int index, traversability_mesh_package::GeoFeature param){
	//* Extract the variables
	// Face
	float final_score=0;
	float denominator=1;
	
	
	return final_score/denominator;
	
}

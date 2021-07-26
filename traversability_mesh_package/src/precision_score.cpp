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
#include<fstream>

using namespace std;

//* Geometry based traverversability score calculator
float prec_calculator(int index, traversability_mesh_package::GeoFeature param){
	//* Extract the variables
	// Face
	float final_score=0;
	float denominator=1;
	float prec_scores[2];
    std::vector<double> example=param.metrics.at(index).features;
	std::ofstream output_file("./mesh_precision_simulation.txt");
    std::ostream_iterator<double> output_iterator(output_file, "\n");
    std::copy(example.begin(), example.end(), output_iterator);
	if(param.metrics.at(index).features[0]>1){
        
        prec_scores[0]=1/(1+param.metrics.at(index).features[1]); // Score regarding the mean of the distance of the points to the mesh elements
        prec_scores[1]=1/(1+param.metrics.at(index).features[2]); // Score regarding the variance of the distance of the points to the mesh elements
        for(int i=0;i<2;i++){
            final_score+=prec_scores[i]*prec_param.at(i);
            denominator+=prec_param.at(i);
        }
        return final_score/denominator;
    }
			
	return 0;
	
}

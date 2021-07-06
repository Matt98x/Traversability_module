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
float geom_calculator(int index, traversability_mesh_package::GeoFeature param,traversability_mesh_package::RobotState rstate){
	//* Extract the variables
	// Face
	float final_score=0;
	float denominator=0;
	
	//ROS_INFO("%d",param.areas.size());
	float distance=0; // Distance from the robot reference system to the face baricenter
	float area=param.areas.at(index); // Area of the face
	float slope=param.slopes.at(index); // Slope of the face
	float scores[5]={0};
	
	// Neighbors
	auto neighbors=param.neighbors.at(index).proximals; //extract the neighbors
	sort( neighbors.begin(), neighbors.end() ); // sort them
  neighbors.erase( unique( neighbors.begin(), neighbors.end() ), neighbors.end() ); // erase all duplicates
	neighbors.erase(neighbors.begin()); // get rid of the zero
	for(auto& element : neighbors) // for each neighbor
    element -= 1; // subtract one to find the correct index
	neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), index), neighbors.end()); //remove the face itself from its neighbors
	int n_neighbors=neighbors.size(); // compute the number of neighbors
	
  // Distances
  
	int temp = rstate.state.size();
	vector<float> feet_d(temp); //initialize the distance array
	vector<float> feet_dz(temp); //initialize the step_height array
	tf::Vector3 temp1;
	tf::Vector3 temp2;
  tf::pointMsgToTF(param.baricenters.at(index),temp1); // Temporary location where we can store the face baricenter position
	// feet to face
	for(int i=0;i<rstate.state.size();i++){ // for each foot 
		tf::pointMsgToTF(rstate.state.at(i).pose.pose.position,temp2); // Temporary location where we can store the foot position
		if(i<1){
			distance=(temp1-temp2).length(); // find the distance between the robot center and the face baricenter
		}else{
		 feet_d.at(i)=(temp1-temp2).length(); //find the distance the foot and the face baricenter
		 feet_dz.at(i)=abs(temp1.getZ()-temp2.getZ()); //find the step heigth between the foot and the face baricenter
		}
	}
	
	
	// Neighbors variables
	vector<float> similarity(n_neighbors); //initialize the similarity array
	vector<float> slopes(n_neighbors); //initialize the slopes array
	vector<float> areas(n_neighbors); //initialize the areas array
  vector<float> neighbor_d(n_neighbors); //initialize the distance array
	vector<float> neighbor_dz(n_neighbors); //initialize the step_height array
	
	for(int i=0;i<n_neighbors;i++){ // Going through every neighbor we compute the variables
		// neighbors to face distance (both absolute and vertical distance)
		tf::pointMsgToTF(param.baricenters.at(i),temp2); // Temporary location where we can store the neighbor baricenter position
		neighbor_d.at(i)=(temp1-temp2).length(); //find the distance the foot and the face baricenter
		neighbor_dz.at(i)=abs(temp1.getZ()-temp2.getZ()); //find the step heigth between the foot and the face baricenter
		// Areas
		areas.at(i)=param.areas[neighbors.at(i)];
		// Slopes
		slopes.at(i)=param.slopes[neighbors.at(i)];
	  // Similarity
		abs(slopes.at(i)-slope) < max_ang_diff ? similarity.at(i)=1 : similarity.at(i)=0; // if the slope difference is less than 20 percent the faces are considered similar
	}
	
	//* Compute the scores
	//float scores[5]={0}; 
	
	
	
	//* Compute number of neighbors score
	scores[0] = float(n_neighbors)/12; // number of neighbors over the theorical maximum over a normal meshes(12)
	 if (scores[0]>1 )
		scores[0]=1; // since there might be complex situations we normalize to get a score between 0 and 1
	//* Compute the distance from the foot score
	// minimum foot distance index
	int mindex=std::min_element(feet_d.begin(), feet_d.end())-feet_d.begin();
	scores[1]=1/(1+feet_d.at(mindex));
	//* Compute the step height score
	scores[2]=1-abs(feet_dz.at(mindex)/step_height_max);
	//* Compute the total area score
	float sum1=0;
	float sum2=0;
	for(int i=0;i<areas.size();i++){
		sum1+=similarity.at(i)*areas.at(i);
		sum2+=areas.at(i);
	}
	
	scores[3]=sum1/sum2;
	//* Compute the slope score
	scores[4]=1-abs(slope/slope_max);
	if(scores[4]<0)
		scores[4]=0;
	//* Compute the total geometry based score
	
	for(int i=0;i<5;i++){
		
		final_score+=scores[i]*geo_param.at(i);
		denominator+=geo_param.at(i);
	}
	//* Return the score
			
	return final_score/denominator;
	
}

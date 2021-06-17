//* Necessary libraries
#include <ros/ros.h> // roslibrary

float sum_geom_param; // sum of all the geometrical param to normalize the score from 0 to 1
float step_height_max; // maximum step height 
float slope_max;// maximum allowed slope
float max_ang_diff; // maximum allowed angular difference for similarity
std::vector<float> gen_param; // score computer parameters to weight different 
std::vector<float> geo_param; // temporary variable to store the parameters for the score computation (order= number of neighbors, distance from the foot, step_height, total area, slope cost)


float geom_calculator(int index, traversability_mesh_package::GeoFeature param,traversability_mesh_package::RobotState rstate);

//* Necessary libraries
#include <ros/ros.h> // roslibrary


#ifndef VARIABLES_H // header guards
#define VARIABLES_H

extern float sum_geom_param; // sum of all the geometrical param to normalize the score from 0 to 1
extern float step_height_max; // maximum step height 
extern float slope_max;// maximum allowed slope
extern float max_ang_diff; // maximum allowed angular difference for similarity
extern std::vector<float> gen_param; // score computer parameters to weight different 
extern std::vector<float> geo_param; // temporary variable to store the parameters for the score computation (order= number of neighbors, distance from the foot, step_height, total area, slope cost)
extern std::vector<float> prec_param; // temporary variable to store the parameters for the precision score computation (order= mean, variance)

#endif

float geom_calculator(int index, traversability_mesh_package::GeoFeature param,traversability_mesh_package::RobotState rstate);
float prec_calculator(int index, traversability_mesh_package::GeoFeature param);
float vis_calculator(int index, traversability_mesh_package::GeoFeature param);
//* Necessary libraries
#include <ros/ros.h> // roslibrary
// Message types
#include <std_msgs/ColorRGBA.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <mesh_msgs/MeshMaterial.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <traversability_mesh_package/RobotState.h>
#include <traversability_mesh_package/GeoFeature.h>
// Thread related functions
#include <std_msgs/Empty.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
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
// Libraries related to synchronization
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
// Score calculation library
#include <score_calculators.h>



//* Setting namespaces and typedef to symplify the typing
using namespace sensor_msgs;
using namespace mesh_msgs;
using namespace nav_msgs;
using namespace traversability_mesh_package;
using namespace std;
using namespace message_filters;
//typedef sync_policies::ApproximateTime<GeoFeature,VisFeature> MySyncPolicy;

//* Variable declaration
ros::Publisher point_pub;
mesh_msgs::TriangleMeshStamped output;
boost::mutex mutex; // mutex to handle the access to the vertex structures array
RobotState state; // most recent localization  of the robotly the three approaches


// PARAMETERS TO BE INSERTED IN A CONFIGURATION FILE AND RETRIEVED IN THE FINEAL VERSION
int n_server=3; // temporary solution to define the number of servers, later to be made in a parameter of the package

//* Funtions

//* Thread functions declaration
void master_thread(traversability_mesh_package::GeoFeature param,traversability_mesh_package::RobotState rstate); // one for each callback
void server_thread(int index, int portion, traversability_mesh_package::GeoFeature param,traversability_mesh_package::RobotState rstate); // multiple for each master thread

//* Callback function for the odometry data
void odomcallback( const traversability_mesh_package::RobotState::ConstPtr& msg){
	state=*msg; // put the incoming message in a global variable
}

//* Callback function for the synchronizer
void callback( const traversability_mesh_package::GeoFeature::ConstPtr& msg)
{
	const traversability_mesh_package::GeoFeature inputGeo=*msg;

  	// spawn another thread
  	boost::thread thread_b(master_thread, inputGeo,state);

}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "score_computer");

	//* Handle for the node
	ros::NodeHandle n;
	std::string robot_state;
	//* Get the value from the parameter server
	n.getParam("/general_weight",gen_param);
	n.getParam("/geometric_weight",geo_param);
	n.getParam("/max_step_height",step_height_max);
	n.getParam("/maximum_slope",slope_max);
	n.getParam("/max_ang_diff",max_ang_diff);
	n.getParam("/robot_state",robot_state);

  //* Subscribe to the localization topic
  ros::Subscriber state_sub = n.subscribe(robot_state, 1000, odomcallback);
	//* Subscribe to the Geometric features data topic
  ros::Subscriber feature_sub = n.subscribe("/GeoFeatures", 1000, callback);

/* Code for when there will be the  visual features
  //* Define the  synchronization policy for the input topics: ApproximateTime is used to have the least possible difference between corresponding timestamps while not imposing a perfect identity which would gravely affect the transmission rate
  message_filters::Subscriber<GeoFeature> geo_sub(n, "/GeoFeatures", 1);
	message_filters::Subscriber<VisFeature> vis_sub(n, "/VisFeatures", 1);
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(50), geo_sub,vis_sub);
  sync.registerCallback(boost::bind(&callback, _1,_2));
*/
	//* Advertize the coordinated data
  point_pub = n.advertise<mesh_msgs::TriangleMeshStamped>("test_msg", 1000);
	ros::spin();

  //exit
  return 0;
}

//* Thread functions definition
void master_thread(traversability_mesh_package::GeoFeature param,traversability_mesh_package::RobotState rstate){
	// Initialize the output
	output.header=param.mesh.header;
	output.header.frame_id="world";
	output.mesh.triangles=param.mesh.mesh_geometry.faces;
	output.mesh.vertices=param.mesh.mesh_geometry.vertices;
	output.mesh.face_materials=vector<mesh_msgs::MeshMaterial>(param.mesh.mesh_geometry.faces.size());
 	// Compute the fortion for each server
	int portion=param.mesh.mesh_geometry.faces.size()/n_server;
  // Initialize the servers
  boost::thread s_t[n_server];
  for(int i=0;i<n_server;i++){
		s_t[i]= boost::thread(boost::bind(server_thread,i,portion,param,rstate));
  }
  // Wait for the servers to finish
	for(int i=0;i<n_server;i++){
		s_t[i].join();
  }

  //* Send the feature message 
  point_pub.publish(output);
		
}


void server_thread(int index, int portion, traversability_mesh_package::GeoFeature param,traversability_mesh_package::RobotState rstate){
	//Variables declaration
	tf::Vector3 sum; // Variable to store the sum from which we can obtain the baricenter dividing by 3
	tf::Vector3 temp1; // Temporary location where we can store points and perform calculations
	tf::Vector3 temp2; // Temporary location where we can store points and perform calculation
  int last=(index+1)*portion; // The last index the server should consider is the one immidiately before the one at which the next server start
  if((index+2)*portion>param.mesh.mesh_geometry.faces.size()){ // If there are no other servers
		last=param.mesh.mesh_geometry.faces.size(); // The last index is the one of the last server 
  }

	
  //* Loop over all the indices given to the server by the master  
  for (int i=index*portion;i<last;i++){
		//* compute the geometry based score
		float geometry_score=geom_calculator(i,param,rstate);
		//* Compute the visual based score
		//ROS_INFO("%f",geometry_score);
		// Compute the color for the mesh element
		output.mesh.face_materials[i].texture_index=i;
		output.mesh.face_materials[i].has_texture=true;
		if (geometry_score <= 0.5){
			output.mesh.face_materials[i].color.r=255;
			output.mesh.face_materials[i].color.g=int(geometry_score*255);
		}else{
			output.mesh.face_materials[i].color.r=int((1-geometry_score)*255);
			output.mesh.face_materials[i].color.g=255;
		}
		output.mesh.face_materials[i].color.b=0;
		output.mesh.face_materials[i].color.a=1;
  } 
}

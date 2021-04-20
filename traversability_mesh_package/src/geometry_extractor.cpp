//* Necessary libraries
#include <ros/ros.h> // roslibrary
// message types
#include <mesh_msgs/MeshGeometryStamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <traversability_mesh_package/Base.h>
#include <boost/bind.hpp>
// Thread related functions
#include <std_msgs/Empty.h>
#include <boost/thread/thread.hpp>
// Tf libraries
#include <tf/tf.h>
#include <tf/transform_datatypes.h>


//* Setting namespaces and typedef to symplify the typing
using namespace sensor_msgs;
using namespace mesh_msgs;
using namespace nav_msgs;
using namespace traversability_mesh_package;




//* Variable declaration
ros::Publisher point_pub;
int n_server=3;


//* Thread functions declaration
void master_thread(traversability_mesh_package::Base param); // one for each callback
void server_thread(int* j); // multiple for each master thread

//* Callback function for the synchronizer
void callback( const traversability_mesh_package::Base::ConstPtr& msg)
{
	traversability_mesh_package::Base input=*msg;
  // spawn another thread
  boost::thread thread_b(master_thread, input);
}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "geometry_extractor");

  //* Handle for the node
  ros::NodeHandle n;
  
	//* Subscribe to the coordinated data topic
  ros::Subscriber point_sub = n.subscribe("base_coord", 1000, callback);
	//* Advertize the coordinated data
  point_pub = n.advertise<sensor_msgs::Image>("test_msg", 1000);
	ros::spin();

  //exit
  return 0;
}

//* Thread functions definition
void master_thread(traversability_mesh_package::Base param){
  
  //* Initialize the output
  //* Divide the vertices among the number of servers
  //* Retrieve the transformation matrix elements 
  
  
  //* Initialize the servers
  boost::thread s_t[n_server];
  for(int i=0;i<n_server;i++){
		s_t[i]= boost::thread(boost::bind(server_thread,&i));
    
  }
  //* Wait for the servers to finish
	for(int i=0;i<n_server;i++){
		s_t[i].join();
  }
  //* Send the feature message 
  
}

void server_thread(int* j){
  //* Loop over all the indices given to the server 
  for (int i=0;i<*j;i++){
    //* Compute the real position of the vertices
	  //* Find the baricenter in the world reference frame
    //* Compute the orientation vector of the element
    //* Compute the slope with the modified arccos algorithm
    //* Compute the area of the element
  } 
}


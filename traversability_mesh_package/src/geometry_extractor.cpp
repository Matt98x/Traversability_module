//* Necessary libraries
#include <ros/ros.h> // roslibrary
// message types
#include <mesh_msgs/MeshGeometryStamped.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <traversability_mesh_package/Base.h>
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
// Libraries related to bits conversion
#include<bits/stdc++.h>


//* Setting namespaces and typedef to symplify the typing
using namespace sensor_msgs;
using namespace mesh_msgs;
using namespace nav_msgs;
using namespace traversability_mesh_package;
using namespace std;



//* Variable declaration
ros::Publisher point_pub;
int n_server=3; // temporary solution to define the number of servers, later to be made in a parameter of the package
boost::mutex mutex; // mutex to handle the access to the vertex structures array

// Vertices class definition
typedef struct Vertices {
  geometry_msgs::Point transmitted; // original position with respect to the lidar
  geometry_msgs::Point transformed; // transformed position with respect to the world reference frame
	bool check; // check  to see whether it has been transformed
	bool constructed = false; // initialize the constructed attribute to false  
  vector<unsigned int> neighbors = vector<unsigned int>(0);  // number of neighbours initialized to zero
} Vertices;


//* Thread functions declaration
void master_thread(traversability_mesh_package::Base param); // one for each callback
void server_thread(int index, int portion); // multiple for each master thread

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

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  stable_sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

//* Thread functions definition
void master_thread(traversability_mesh_package::Base param){
  

  //* Retrieve the transformation matrix elements 

	// Transformation from origin to trunk configuration
  geometry_msgs::Point translation = param.odom.pose.pose.position; // retrieve the translation from the world origin to the trunk reference system
  geometry_msgs::Quaternion rotation = param.odom.pose.pose.orientation; // retrieve the quaternion of the trunk reference system w.r.t. the world reference system

	// Transformation from trunk to LiDAR
  /* ..Can be hardcoded or passed as a parameter, the most important feature is that is constant.. */

	//* Initialize the output
	traversability_mesh_package::GeoFeature output; // the output is the mesh plus the geometrical features that will be extracted now

	//* Initialize the vertices array and give compute all faces neighboring to each vertex
  
	//We declare same variables to make the computation easier
	mesh_msgs::MeshGeometry mesh = param.mesh.mesh_geometry; // extraction of the mesh
	int n_vertices=sizeof(mesh.vertices)/sizeof(mesh.vertices[0]); //number of vertices
	int n_faces = sizeof(mesh.faces)/sizeof(mesh.faces[0]); // number of faces
  int p_index; // index of the vertex we are considering at each instant
	Vertices vertices[n_vertices]; // Initialization of the vertices array

 	// We perform the loop to find all neighboring faces of each vertex
  for(int i=0; i<n_faces;i++){ // for all faces
		for(int j;j<3;j++){ // for all vertices of a face
			p_index = mesh.faces[i].vertex_indices[j]; // index assignment
 			if(!vertices[p_index].constructed){ // if the  vertex have not been constructed yet
				vertices[p_index].transmitted=mesh.vertices[p_index]; // input the vertex coordinates received from the previous node
				vertices[p_index].constructed=true; // Impose that the vertex has been initialized 
				vertices[p_index].neighbors.at(0)=i; // say that the first neighboring face is the one at which the face has been initialized
			}else{
				vertices[p_index].neighbors.push_back(i); // add the new face to the list of neighboring faces
			}
		}
	}
  
  //* Divide the faces among the number of servers

  // compute how many faces should go to each server
	unsigned int portion = n_faces/n_server; // how many faces the server should handle
	
  // Initialize the servers
  boost::thread s_t[n_server];
  for(int i=0;i<n_server;i++){
		s_t[i]= boost::thread(boost::bind(server_thread,i,portion));
  }
  // Wait for the servers to finish
	for(int i=0;i<n_server;i++){
		s_t[i].join();
  }

  //* Send the feature message 
  
}

void server_thread(int index, int portion){
  //* Loop over all the indices given to the server by the master
  for (int i=index*portion;i<(index+1)*portion;i++){
    //* Compute the real position of the vertices
		for(int j=0;j<y3;j++){ // for each vertex
		}
	  //* Find the baricenter in the world reference frame
    //* Compute the orientation vector of the element
    //* Compute the slope with the modified arccos algorithm
    //* Compute the area of the element
		//* Find the proximals
  } 
}


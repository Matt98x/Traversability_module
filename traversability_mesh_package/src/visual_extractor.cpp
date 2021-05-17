//* Necessary libraries
#include <ros/ros.h> // roslibrary
// Message types
#include <mesh_msgs/MeshGeometryStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <traversability_mesh_package/Data.h>
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


//* Setting namespaces and typedef to symplify the typing
using namespace sensor_msgs;
using namespace mesh_msgs;
using namespace nav_msgs;
using namespace traversability_mesh_package;
using namespace std;

//* Variable declaration
ros::Publisher output_pub;
int n_server=1; // temporary solution to define the number of servers, later to be made in a parameter of the package

boost::mutex mutex1;
vector<Data> List(10);
traversability_mesh_package::GeoFeature output; // the output is the mesh plus the geometrical features that will be extracted now
traversability_mesh_package::GeoFeature reset;

// Vertices class definition
typedef struct Vertices {
  geometry_msgs::Point transmitted; // original position with respect to the lidar
  geometry_msgs::Point transformed; // transformed position with respect to the world reference frame
	bool check = false; // check  to see whether it has been transformed
	bool constructed = false; // initialize the constructed attribute to false 
	int index=0;  // index on which the neighbor is written
  vector<unsigned int> neighbors=vector<unsigned int>(20);  // number of neighbours initialized to one
} Vertices;


//* Thread functions declaration
void master_thread(traversability_mesh_package::Data param); // one for each callback
void server_thread(int index, int portion, Vertices *vertices, tf::Transform transform, tf::Vector3 sight_line, boost::mutex *mutex); // multiple for each master thread

//* Function to more accurately compute the acos of some dot product
float acosine(float theta){
	double a = -0.939115566365855;
	double b =  0.9217841528914573;
	double c = -1.2845906244690837;
	double d =  0.295624144969963174;
	return M_PI/2 + (a*theta + b*theta*theta*theta) / (1 + c*theta*theta + d*theta*theta*theta*theta);
}

//* Callback function for the subscriber
void callback( const traversability_mesh_package::Data::ConstPtr& msg)
{
	traversability_mesh_package::Data input=*msg;
	
  // spawn another thread
	boost::thread thread_b(master_thread,input);

}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "geometry_extractor");

  //* Handle for the node
  ros::NodeHandle n;
  
	//* Subscribe to the coordinated data topic
  ros::Subscriber input_sub = n.subscribe("base_coord", 10, callback);
	//* Advertize the coordinated data
  output_pub = n.advertise<GeoFeature>("/GeoFeatures", 1000);

	
	ros::spin();

  //exit
  return 0;
}

//* Thread functions definition
void master_thread(traversability_mesh_package::Data param){
  

  //* Retrieve the transformation matrix elements 

	// Transformation from origin to trunk configuration
  geometry_msgs::Transform T_wt;
	// retrieve the translation from the world origin to the trunk reference system
  T_wt.translation.x=param.odom.pose.pose.position.x;
	T_wt.translation.y=param.odom.pose.pose.position.y;
	T_wt.translation.z=param.odom.pose.pose.position.z;
	// retrieve the quaternion of the trunk reference system w.r.t. the world reference system
  T_wt.rotation=param.odom.pose.pose.orientation;

	// Transformation from trunk to LiDAR
  /* ..Can be hardcoded or passed as a parameter(in the final version it will be like this), the most important feature is that is constant.. */
	geometry_msgs::Transform T_tl;
  T_tl.translation.x=0.267; // translation of the LiDar along the x-axis
  T_tl.translation.y=0.000; // translation of the LiDar along the y-axis
	T_tl.translation.z=0.000; // translation of the LiDar along the z-axis
  geometry_msgs::Quaternion const_rot; // retrieve the quaternion of the LiDAR reference system w.r.t. the trunk reference system
  const_rot.x=-0.500; // rotation of the LiDar along the x-axis
  const_rot.y=0.500; // rotation of the LiDar along the y-axis
	const_rot.z=-0.500; // rotation of the LiDar along the z-axis
  const_rot.w=0.500; // rotation of the LiDar along the w-axis
 	T_tl.rotation=const_rot;

  // Compute the final transformation matrix from LiDAR to world (T[trunk to world]*T[LiDAR to trunk])
  tf::Transform T_w, T_t, res; // Prepare tf::Transform to perform the calculations
  tf::transformMsgToTF(T_wt,T_w); // Transform the world to trunk in a transform
	tf::transformMsgToTF(T_tl,T_t); // Transform the trunk to LiDAR in a transform
	res.mult(T_w,T_t); // Multiply the two transforms and store them in the resulting final transform (This will be used to compute the lidar point position with respect to the world)

	// We want to use the trunk to world transformation to generate the observation direction of the robot
	// we set the translation to 0
  T_wt.translation.x=0;
	T_wt.translation.y=0;
	T_wt.translation.z=0;
	// Generate the new transformation matrix
	tf::transformMsgToTF(T_wt,T_w); // Transform the world to trunk in a transform

  tf::Vector3 temp; // Temporary location where we can store the x-versor and perform calculations
	temp.setX(1);
	temp=T_w.inverse()*temp; // find the x direction of the robot with respect to the world reference frame
	temp.normalize();
	//* Initialize the vertices array and give compute all faces neighboring to each vertex
  
//* Initialize the output
	//We declare same variables to make the computation easier

	int n_vertices = param.mesh.mesh_geometry.vertices.size(); //number of vertices
	int n_faces = param.mesh.mesh_geometry.faces.size(); // number of faces
  int p_index; // index of the vertex we are considering at each instant
	Vertices vertices[n_vertices]; // Initialization of the vertices array


  // Generates arrays of appropriate dimension
	
	output.header=param.mesh.header;
	output.mesh=param.mesh;
  output.areas=vector<double>(n_faces);;
	output.slopes=vector<double>(n_faces);
	output.normals=vector<geometry_msgs::Point>(n_faces);
	output.baricenters=vector<geometry_msgs::Point>(n_faces);
  output.neighbors=vector<traversability_mesh_package::Proximals>(n_faces);
	// sight direction (corrisponding to the robot x direction in the world reference frame)
	output.sight_dir.x=temp.getX();
	output.sight_dir.y=temp.getY();
	output.sight_dir.z=temp.getZ(); 


 	// We perform the loop to find all neighboring faces of each vertex
  for(int i=0; i<n_faces;i++){ // for all faces
		for(int j=0;j<3;j++){ // for all vertices of a face
			p_index = param.mesh.mesh_geometry.faces.at(i).vertex_indices[j]; // index assignment
 			if(!vertices[p_index].constructed){ // if the  vertex have not been constructed yet
				vertices[p_index].transmitted=output.mesh.mesh_geometry.vertices[p_index]; // input the vertex coordinates received from the previous node
				vertices[p_index].constructed=true; // Impose that the vertex has been initialized 
				vertices[p_index].neighbors[vertices[p_index].index]=i; // say that the first neighboring face is the one at which the face has been initialized
				vertices[p_index].index++; // Increment the writing index
			}else{
				vertices[p_index].neighbors[vertices[p_index].index]=i; // say that the first neighboring face is the one at which the face has been initialized
				vertices[p_index].index++; // Increment the writing index
			}
		}
	}
  
  //* Divide the faces among the number of servers
	boost::mutex mutex;
	boost::thread s_t[n_server];
  // compute how many faces should go to each server
	unsigned int portion = n_faces/n_server; // how many faces the server should handle
  for(int i=0; i<n_server;i++)
			s_t[i]=boost::thread(server_thread,i,portion,vertices,res,temp, &mutex);

	for(int  i=0;i<n_server;i++)
			s_t[i].join();
  GeoFeature message;


  //* Send the feature message 
	output_pub.publish(output);

}

void server_thread(int index, int portion, Vertices *vertices, tf::Transform transform, tf::Vector3 sight_line, boost::mutex *mutex){

	//Variables declaration
	tf::Vector3 sum; // Variable to store the sum from which we can obtain the baricenter dividing by 3
	tf::Vector3 temp1; // Temporary location where we can store points and perform calculations
	tf::Vector3 temp2; // Temporary location where we can store points and perform calculation
  int last=(index+1)*portion; // The last index the server should consider is the one immidiately before the one at which the next server start
	int a=0;
  if((index+2)*portion>output.mesh.mesh_geometry.faces.size()){ // If there are no other servers
		a=last;
		last=output.mesh.mesh_geometry.faces.size(); // The last index is the one of the last server
  }
	vector<geometry_msgs::Point> temporary=vector<geometry_msgs::Point>(output.mesh.mesh_geometry.vertices.size());
	geometry_msgs::Point temporary1;

  //* Loop over all the indices given to the server by the master  
  for (int i=index*portion;i<last;i++){
	
		output.neighbors[i].proximals=vector<unsigned int>(20);
    //* Compute the real position of the vertices
		mesh_msgs::TriangleIndices cur = output.mesh.mesh_geometry.faces[i]; // store the current face
		sum*=0;
		vector<unsigned int> neighbors = vector<unsigned int>(100);
    vector<unsigned int> temps;
		vector<unsigned int> itself = vector<unsigned int>(1);
		int partial=0;
		
		for(int j=0;j<3;j++){ // for each vertex

      if(vertices[cur.vertex_indices[j]].check){ // If the point have already been converted in the world reference space
				// Retrieve the position of the point in the world reference system
				temp1.setX(vertices[cur.vertex_indices[j]].transformed.x);
				temp1.setY(vertices[cur.vertex_indices[j]].transformed.y);
				temp1.setZ(vertices[cur.vertex_indices[j]].transformed.z);
			}else{ // Otherwise
			
				while(1){ // We enter a infinite loop to check the situation of the shared memory
					if(mutex->try_lock()){ // if you can take control of the mutex	
						// Store the point in a Vector3
						temp2.setX(vertices[cur.vertex_indices[j]].transmitted.x);
						temp2.setY(vertices[cur.vertex_indices[j]].transmitted.y);
						temp2.setZ(vertices[cur.vertex_indices[j]].transmitted.z);
						// Transform the point to the world reference fame
						temp1=transform*temp2;
						// Store the transformed point in the correct place 
						vertices[cur.vertex_indices[j]].transformed.x=temp1.getX();
						vertices[cur.vertex_indices[j]].transformed.y=temp1.getY();
						vertices[cur.vertex_indices[j]].transformed.z=temp1.getZ();
						// Save the new point in the output
						tf::pointTFToMsg (temp1,output.mesh.mesh_geometry.vertices[cur.vertex_indices[j]]);

						vertices[cur.vertex_indices[j]].check=true; // Set the transformation check to true
						mutex->unlock(); // Unlock the mutex so that other can modify other points if they need
						break;
					}else{ // try to see if the thread controlling the mutex is changing the vertex you are interested in
						if(vertices[cur.vertex_indices[j]].check){ // if it has modified the vertex you are interested in, you just need to read the transformed point 
							temp1.setX(vertices[cur.vertex_indices[j]].transformed.x);
							temp1.setY(vertices[cur.vertex_indices[j]].transformed.y);
							temp1.setZ(vertices[cur.vertex_indices[j]].transformed.z);
							break;
						}
					}
				}				
			}
    
        
				for(unsigned int x : vertices[cur.vertex_indices[j]].neighbors) // if you want to add 10 to each element
    					x ++;
				
				output.neighbors[i].proximals.insert(output.neighbors[i].proximals.end(), vertices[cur.vertex_indices[j]].neighbors.begin(), vertices[cur.vertex_indices[j]].neighbors.end());
				partial+=vertices[cur.vertex_indices[j]].index; 
			  sum+=temp1; // Update the sum of the vertices positions
		}


	  //* Find the baricenter in the world reference frame
		sum/=3; // Divide by 3 to obtain the avarage
		output.baricenters[i].x=sum.getX();
		output.baricenters[i].y=sum.getY();
		output.baricenters[i].z=sum.getZ();

		//* Compute two vectors to compute all the other features of the face (generating taking the vertices 1 and 2 and subtracting the vertex at index 0)
		temp1.setX(vertices[cur.vertex_indices[1]].transformed.x-vertices[cur.vertex_indices[0]].transformed.x);
		temp1.setY(vertices[cur.vertex_indices[1]].transformed.y-vertices[cur.vertex_indices[0]].transformed.y);
		temp1.setZ(vertices[cur.vertex_indices[1]].transformed.z-vertices[cur.vertex_indices[0]].transformed.z);
		temp2.setX(vertices[cur.vertex_indices[2]].transformed.x-vertices[cur.vertex_indices[0]].transformed.x);
		temp2.setY(vertices[cur.vertex_indices[2]].transformed.y-vertices[cur.vertex_indices[0]].transformed.y);
		temp2.setZ(vertices[cur.vertex_indices[2]].transformed.z-vertices[cur.vertex_indices[0]].transformed.z);

		//* Compute the orientation vector of the element

		sum=temp1.cross(temp2); // the orientation vector will be the cross product between the two sides
		if(sum.dot(sight_line)>0) // we want to correct the orientation since from the mesh reconstruction all mesh elements normals should point to the robot
			sum*=-1; // With this the dot product is always negative, which mean that the normal is pointing in the direction of the robot
	  // Of this we compute the length since it gives hints for the computation of the area and to find the versor
		float L=sum.length(); // Here we find the length of the orientation vector
		temp1=sum/L; // With this we normalize the vector making temp1 the versor of sum
		// Input the normals in the output
    output.normals[i].x=temp1.getX();
		output.normals[i].y=temp1.getY();
		output.normals[i].z=temp1.getZ();
    //* Compute the slope with the modified arccos algorithm
    float alength =temp1.getZ(); // with this we find the dot product of the orientation with the vertical axis; From this we can compute the slope of the mesh element
		if(!isnan(alength)){	 // If the normal is defined	
			float slope = acosine(alength); // compute the slope as the arcosine of the the dot product
			output.slopes[i]=slope; // fill the slope field
		}else{
			output.slopes[i]=M_PI; // we put the maximum possible slope
		}
    //* Compute the area of the element
		float area = L/2; // The area of the mesh element is simply the normal divided by 2
		output.areas[i]=area; // fill the area field
		
  } 
}


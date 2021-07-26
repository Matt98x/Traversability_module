//* Necessary libraries
#include <ros/ros.h> // roslibrary
// Message types
#include <mesh_msgs/MeshGeometryStamped.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <traversability_mesh_package/Data.h>
#include <traversability_mesh_package/GeoFeature.h>
#include <traversability_mesh_package/Features.h>
#include <traversability_mesh_package/AccMetrics.h>
// Thread related functions
#include <std_msgs/Empty.h>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
// Tf libraries
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
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
#include <list>
#include <bits/stdc++.h>
// Opencv libraries
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdlib>
#include <opencv/cv.h>
#include <opencv/highgui.h>
// pointcloud search
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/passthrough.h>

//* Setting namespaces and typedef to symplify the typing
using namespace sensor_msgs;
using namespace mesh_msgs;
using namespace nav_msgs;
using namespace traversability_mesh_package;
using namespace std;

//* Variable declaration
ros::Publisher output_pub;
//tf2_ros::Buffer tfBuffer;
//tf2_ros::TransformListener tfListener(tfBuffer);
int n_server=2; // temporary solution to define the number of servers, later to be made in a parameter of the package
float k=0.9; //Size of the voxel used to downsample the pointcloud [m] increasing this increases the trasmission rate while reducing the precision of the consequent mesh
int clip;
boost::mutex mutex1;
vector<Data> List(10);
traversability_mesh_package::GeoFeature output; // the output is the mesh plus the geometrical features that will be extracted now
traversability_mesh_package::GeoFeature reset;

// Vertices class definition
typedef struct cloudpoint {  
  vector<unsigned int> neighbors=vector<unsigned int>(0);  // number of neighbours initialized to one
} cloudpoint;

// Functions to see if the the point is inside the face perimeter
bool SameSide(tf::Vector3 p,tf::Vector3 a,tf::Vector3 b,tf::Vector3 c){
	tf::Vector3 cp1=(c-b).cross(p-b);
	tf::Vector3 cp2=(c-b).cross(a-b);
	if(cp1.dot(cp2)>=0){
		return true;
	}
	return false;
}

bool PointInTriangle(tf::Vector3 p,tf::Vector3 a,tf::Vector3 b,tf::Vector3 c){
	if(SameSide(p,a,b,c) && SameSide(p,b,a,c) && SameSide(p,c,a,b) )
		return true;
	return false;
}

// Function to check whether a point really correspond to the measure of the face closer to it
bool check_point(int index,mesh_msgs::MeshGeometry geometry,pcl::PointXYZ searchPoint){
	// Variables definition
	vector<tf::Vector3> vertices=vector<tf::Vector3>(3); // Declaration of the three vertices of the face	
	tf::Vector3 sideA,sideB; // side of the face
	tf::Vector3 projection;
	tf::Vector3 mainP;
	// Set the point
	mainP.setX(searchPoint.x);
	mainP.setY(searchPoint.y);
	mainP.setZ(searchPoint.z);
	for(int i=0;i<3;i++){ // for each vertex
		// Set the coordinate
		vertices.at(i).setX(geometry.vertices.at(geometry.faces.at(index).vertex_indices.at(i)).x);
		vertices.at(i).setY(geometry.vertices.at(geometry.faces.at(index).vertex_indices.at(i)).y);
		vertices.at(i).setZ(geometry.vertices.at(geometry.faces.at(index).vertex_indices.at(i)).z);
	}
	sideA=vertices.at(1)-vertices.at(0); // first side of the face
	sideB=vertices.at(2)-vertices.at(0); // second side of the face
	// Find the projection of the point on the face plane
	projection=vertices.at(0)+mainP.dot(sideA)*sideA+mainP.dot(sideB)*sideB; // projection of the point onthe face
	// See if the projection is inside the face
	return PointInTriangle(projection,vertices.at(0),vertices.at(1),vertices.at(2));

}

// Function to compute the distance of the points to the face
float f_distance(int index,mesh_msgs::MeshGeometry geometry,pcl::PointXYZ searchPoint){
	// Variables definition
	vector<tf::Vector3> vertices=vector<tf::Vector3>(3); // Declaration of the three vertices of the face	
	tf::Vector3 sideA,sideB; // side of the face
	tf::Vector3 projection;
	tf::Vector3 mainP;
	// Set the point
	mainP.setX(searchPoint.x);
	mainP.setY(searchPoint.y);
	mainP.setZ(searchPoint.z);
	for(int i=0;i<3;i++){ // for each vertex
		// Set the coordinate
		vertices.at(i).setX(geometry.vertices.at(geometry.faces.at(index).vertex_indices.at(i)).x);
		vertices.at(i).setY(geometry.vertices.at(geometry.faces.at(index).vertex_indices.at(i)).y);
		vertices.at(i).setZ(geometry.vertices.at(geometry.faces.at(index).vertex_indices.at(i)).z);
	}
	sideA=vertices.at(1)-vertices.at(0); // first side of the face
	sideB=vertices.at(2)-vertices.at(0); // second side of the face
	return float((mainP-vertices.at(0)).dot(sideA.cross(sideB)));

}

// Master thread function
void master_thread(traversability_mesh_package::Data param){
  
    // Initialize some variable
	int n_vertices = param.mesh.mesh_geometry.vertices.size(); //number of vertices
	int n_faces = param.mesh.mesh_geometry.faces.size(); // number of faces
    int p_index; // index of the vertex we are considering at each instant

    //* Initialize the output	
	output.header=param.mesh.header;
	output.mesh=param.mesh;
	output.areas=vector<double>(n_faces);;
	output.slopes=vector<double>(n_faces);
	output.normals=vector<geometry_msgs::Point>(n_faces);
	output.baricenters=vector<geometry_msgs::Point>(n_faces);
	output.neighbors=vector<traversability_mesh_package::Proximals>(n_faces);
	output.face_features=vector<traversability_mesh_package::Features>(n_faces);
	traversability_mesh_package::AccMetrics initl;
	initl.features=vector<double>(3,0.0);
	output.metrics=vector<traversability_mesh_package::AccMetrics>(n_faces, initl);

	// OCTREE OF FACES BARICENTERS CREATION
	// Transform the pointcloud from a ros sensor message to a pcl object
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width=n_faces;
	cloud->height=1;
	cloud->points.resize(cloud->width*cloud->height);
	//ROS_INFO("%d %d",cloud->size(),n_faces);
	for(std::size_t i=0; i<n_faces; i++){
		float baricenter_x=0.0,baricenter_y=0.0,baricenter_z=0.0; // initialization of the baricenter coordinates
		for(int j=0;j<3;j++){ // Find the baricenter of the face without dividing(sum the respective coordinates of the three vertices)
			baricenter_x+=param.mesh.mesh_geometry.vertices.at(param.mesh.mesh_geometry.faces.at(i).vertex_indices[j]).x;
			baricenter_y+=param.mesh.mesh_geometry.vertices.at(param.mesh.mesh_geometry.faces.at(i).vertex_indices[j]).y;
			baricenter_z+=param.mesh.mesh_geometry.vertices.at(param.mesh.mesh_geometry.faces.at(i).vertex_indices[j]).z;
		}
		(*cloud).at(i).x = baricenter_x/3.0; // assign the new point x-coordinate while dividing by 3(number of vertices of the face)
    	(*cloud).at(i).y = baricenter_y/3.0; // assign the new point y-coordinate while dividing by 3(number of vertices of the face)
    	(*cloud).at(i).z = baricenter_z/3.0; // assign the new point z-coordinate while dividing by 3(number of vertices of the face)
		/*
		output.metrics.at(i).features.insert(output.metrics.at(i).features.end(),0.0); // point counter
		output.metrics.at(i).features.insert(output.metrics.at(i).features.end(),0.0); // mean extractor
		output.metrics.at(i).features.insert(output.metrics.at(i).features.end(),0.0); // variance extractor
		output.metrics.at(i).features[2]=0.0; // variance extractor
		*/
		vector<double> var1=vector<double>(3,0.0);
		output.metrics.at(i).features=var1;
		output.metrics.at(i).features[0]=0.0;
		output.metrics.at(i).features[1]=0.0;
		output.metrics.at(i).features[2]=0.0;
	}

	// Define the resolution of the octree(the number of elements in the leafs)
	float resolution = 128.0f;

	// Initialize the Octree
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
	// Put the baricenters in the Octree
	octree.setInputCloud (cloud);
	octree.addPointsFromInputCloud ();
	
	// SUBSAMPLING
	// Take a subsample of the points in the pointcloud 
	pcl::PCLPointCloud2* cloud_in = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud_in);
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

	//* Convert to PCL data type
	pcl_conversions::toPCL(param.points, *cloud_in);

	//* Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (k*0.1, k*0.1, k*0.1);
	sor.filter (*cloud_filtered); 
	if(0){
		pcl::PassThrough<pcl::PCLPointCloud2> sor1;
		sor1.setInputCloud (cloudPtr);
		sor1.setFilterFieldName ("z");
		sor1.setFilterLimits(0.5,3.0);
		sor1.filter (*cloud_filtered);
	}
	double templ;
	// POINTS TO FACE MATCHING
	list<cloudpoint> NList= list<cloudpoint>(n_faces);
	pcl::PointCloud<pcl::PointXYZ>::Ptr vertices( new pcl::PointCloud<pcl::PointXYZ> );
	pcl::fromPCLPointCloud2( *cloud_filtered, *vertices );
	traversability_mesh_package::Proximals initializer=traversability_mesh_package::Proximals();
	initializer.proximals=std::vector<unsigned int>(1);
	vector<traversability_mesh_package::Proximals> face_neighbors=vector<traversability_mesh_package::Proximals>(n_faces,initializer);
	// Loop over all points in the pointcloud
	for(int i=0;i<vertices->size();i++){
		
		pcl::PointXYZ searchPoint = vertices->points[ i ];

		// Find the index of the closest neighbor
		int K = 10; // Find the closest 10 points 
		std::vector<int> pointIdxNKNSearch; // initilize the index
		std::vector<float> pointNKNSquaredDistance; // initialize the square distance
		 
		if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  		{	
			// Extract the index of the closest face to the query point
			unsigned int index=pointIdxNKNSearch[0];
			/*for(int q=0;q<pointIdxNKNSearch.size();q++)
				cout << pointIdxNKNSearch[q] << " ";
			cout << endl;*/
			float distance;
			//ROS_INFO("%d",index);
			// Verify that the projection along the normal intersect the face
			if(check_point(index,param.mesh.mesh_geometry,searchPoint)){
				// Add the index of the point to the list of points of the face
				if(index>=0 && index<output.metrics.size()){
					
					//ROS_INFO("%d",index);
					// Update the metrics for the face
					if(output.metrics.at(index).features.size()>2){
						
						
						//ROS_INFO("%lf %d %d",output.metrics.at(index).features[0], index,n_faces);
						templ = output.metrics.at(index).features.at(0)+1.0;
						output.metrics.at(index).features.at(0)=templ; // update the counter
							
						//templ=output.metrics.at(index).features.at(1)+f_distance(index,param.mesh.mesh_geometry,searchPoint);
						//output.metrics.at(index).features.at(1)=templ; // add to the counter the distance of the face from the point
					}
					face_neighbors.at(index).proximals.push_back(i);
					
					

				}
				
			}
			
  		}
		
	}
	
	
	
	// METRICS COMPUTATIONS
	// For all faces computes the metrics
	/*for(int i=0;i<n_faces;i++){
		face_neighbors.at(i).proximals.erase(face_neighbors.at(i).proximals.begin());	

		if(output.metrics.at(i).features.at(0)>0){
			output.metrics.at(i).features.at(1)/=output.metrics.at(i).features[0]; // compute the mean 
			for(int j=0;j<face_neighbors.at(i).proximals.size();j++){
				pcl::PointXYZ searchPoint = vertices->points[face_neighbors.at(i).proximals.at(j)]; // generate the point in the list
				float d=f_distance(i,param.mesh.mesh_geometry,searchPoint); // compute the distance
				output.metrics.at(i).features[2]+=(d-output.metrics.at(i).features[1])*(d-output.metrics.at(i).features[1]); // compute the sqare of the distance from the mean
			}
			output.metrics.at(i).features[2]/=(output.metrics.at(i).features[0]-1); // finalize the computation of the mean
		}
	}
	*/
    //* Send the feature message 
	output_pub.publish(output);
	
}

//* Callback function for the subscriber
void callback( const traversability_mesh_package::Data::ConstPtr& msg)
{
	traversability_mesh_package::Data input=*msg;
	try{
		// spawn another thread
		boost::thread thread_b(master_thread,input);
	}catch(...){
		ROS_INFO("A problem occurred");
	}
  	

}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "points_extractor");
	
  //* Handle for the node
  ros::NodeHandle n;
  n.getParam("handler_precision",k);
	n.getParam("clipper",clip);
	//* Subscribe to the coordinated data topic
  ros::Subscriber input_sub = n.subscribe("base_coord", 10, callback);
	//* Advertize the coordinated data
  output_pub = n.advertise<GeoFeature>("/PointFeatures", 1000);

	ros::spin();

  //exit
  return 0;
}

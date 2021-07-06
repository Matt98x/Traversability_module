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



//* Setting namespaces and typedef to symplify the typing
using namespace sensor_msgs;
using namespace mesh_msgs;
using namespace nav_msgs;
using namespace traversability_mesh_package;
using namespace std;

//* Variable declaration
ros::Publisher output_pub;
int n_server=2; // temporary solution to define the number of servers, later to be made in a parameter of the package
int threshold=10; // threshold for the computation on the features(applied only for triangles with height exceeding the value of the threshold)
vector<float> P; // transformation from camera space to pixel space
vector<float> transform_lc; // transformation from lidar to camera 
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
  vector<unsigned int> neighbors=vector<unsigned int>(1);  // number of neighbours initialized to one
} Vertices;

// Histogram class
struct histogram{
	int intensity;
	int counter;
};

// Color class definition
struct RGB {
    uchar blue;
    uchar green;
    uchar red;  };

// Face features extraction
void features_extraction(list<RGB> colors, float *features){
	//variables declaration 
	int mr=0,mb=0,mg=0;
	list<histogram> p=list<histogram>(0);
	int zi=0,k=0,counter=0;
	
	// Features extraction
	// First we compute the mean red,green, blue and intensity value, along with the histogram of each intensity value and the length of the histogram
	for(RGB c : colors){
		mr+=c.red; // adder for the red value
		mg+=c.green; // adder for the red value
		mb+=c.blue; // adder for the red value
		k=0;
		zi=int(sqrt((c.red*c.red)+(c.green*c.green)+(c.blue*c.blue))); // intensity value of the pixel
		
		list<histogram>::iterator f = p.begin();
    	for(int i=0; i<p.size(); i++){
        	++f;
			if(abs((*f).intensity-zi)<0.25){
				(*f).counter++; // add to the counter
				k=1; // say that the the item is already in the list
				//break;
			}
			//ROS_INFO("%d %d %d",(*f).counter,(*f).intensity,zi);
    	}
		
		if(k<1 || p.size()<1){ // if the item was not in the list
			histogram temp; // initialize a temporary variable for the instagram
			temp.intensity=zi; // set the intensity
			temp.counter=1; // set the counter to 1
			p.insert(p.end(),temp); // Put the element in the list
		}		
		counter++;
	}
	

	features[0]=float(mr)/float(counter); // mean r
	features[1]=float(mb)/float(counter); // mean blue
	features[2]=float(mg)/float(counter); // mean green
	
	// Now we can compute the rest of the features
	for(histogram i : p){
		if(i.counter!=0){
			features[3]+=i.intensity*float(i.counter)/float(counter); //mean intensity value
		}
	}
	for(histogram i : p){
		if(i.counter!=0){
			features[4]+=(i.intensity-features[3])*(i.intensity-features[3])*float(i.counter)/float(counter); // variance of the grey image
			features[6]+=(i.intensity-features[3])*(i.intensity-features[3])*(i.intensity-features[3])*float(i.counter)/float(counter);; // skewness of the intensity histogram
			features[7]+=(float(i.counter)/float(counter))*float(i.counter)/float(counter); // uniformity
			features[8]+=(float(i.counter)/float(counter))*log2(float(i.counter)/float(counter)); // entropy or randomness for the gray levels
		}
	}
	features[4]=sqrt(features[4]); // compute the standard deviation from the variance
	features[5]=features[4]/(1+features[4]); // smoothness of the gray image
	features[8]=-features[8]; // put the negative sign
}

// Function to obtain the pixel indices of a face
int addPixels(list<int>& pixels,int* pixely,int* pixelx,int indtop,int indbottom, float m3, int temp, int ind3,int width, int k){
	int number=-(pixely[indtop]-pixely[indbottom]); // number of rows occupied by the portion of mesh element in the image
	int x_min,x_max; // minimum and maximum x-coordinates at a row y
	float mi; // slope of the lines connecting the top vertex of the portion to the bottom
	if(number==0){ // If the number is zero
		return 0; // skip the portion
	}
	// ROS_INFO("%d %d %d %d %d %d",pixelx[indtop],pixely[indtop],pixelx[indbottom],pixely[indbottom],pixelx[3-indtop-indbottom],pixely[3-indtop-indbottom]);
	for(int i=0;i<number;i++){ // Going over all rows er find the maximum and minimum x values of the portionn at that row
		if(abs(pixelx[indtop]-pixelx[indbottom])>0){ // if the line is not vertical
			mi=float(pixely[indtop]-pixely[indbottom])/float(pixelx[indtop]-pixelx[indbottom]); // compute the slope of the main side
			x_min=(i-number)/mi+pixelx[indbottom]; // compute the limit on the main side
		}else{ // otherwise
			x_min=pixelx[indtop];  // compute the limit on the main side
		}
		 
		
		if(temp==1){ // the line is vertical
			x_max=m3; // comput the limit on the other side
		}else{ // otherwise
			x_max=(i-number)/m3+pixelx[ind3];; // compute the other border
		}
		// If the left and right extreme are switched, we switch them back
		int tempreorder;
		if(x_min>x_max){
			tempreorder=x_min;
			x_min=x_max;
			x_max=tempreorder;
		}
		//ROS_INFO("%d %d",x_max,x_min); //debugging code
		// We insert all the pixels between the two extremes giving their code 
		for(int j=x_min;j<x_max+1;j++){
			int index=(pixely[indtop]+i)*width+j; // find the index of the pixel in the row
			if(k==0){
				pixels.front()=index; // for the fist elemet simply give the correct value
				k=1; // signal that this is not the first element anymore 
			}else{
				pixels.insert(pixels.end(),index); // add the index to the list of pixels related to the pixels list
			}
		}
		
		
	}
	//ROS_INFO("\n"); // debugging code
	//ROS_INFO("%d\n",number);
	return 1;
}

//* Thread functions declaration
void  server_thread(int index, int portion, Vertices *vertices,traversability_mesh_package::Data param,cv::Mat image,float f, tf::Transform transform1,tf::Transform transform2, boost::mutex *mutex){

	//Variables declaration
	tf::Vector3 sum; // Variable to store the sum from which we can obtain the baricenter dividing by 3
	tf::Vector3 temp1; // Temporary location where we can store points and perform calculations
	tf::Vector3 temp2; // Temporary location where we can store points and perform calculation
  int last=(index+1)*portion; // The last index the server should consider is the one immidiately before the one at which the next server start
  if((index+2)*portion>output.mesh.mesh_geometry.faces.size()){ // If there are no other servers
		last=output.mesh.mesh_geometry.faces.size(); // The last index is the one of the last server
  }
	vector<geometry_msgs::Point> temporary=vector<geometry_msgs::Point>(output.mesh.mesh_geometry.vertices.size()); 
	geometry_msgs::Point temporary1;

	float min_,max_;
	int indmin=0;
	int indmax=0;
	float meanx=0.0;
	float meany=0.0;
	int indmed=0;
  //* Loop over all the indices given to the server by the master  
    for (int i=index*portion;i<last;i++){
		float x_array[3]; // Array to contain the x-image-coordinate for the face
		float y_array[3]; // Array to contain the y-image-coordinate for the face
		int xarray[3]; // Array to contain the x-pixel-coordinate for the face
		int yarray[3]; // Array to contain the y-pixel-coordinate for the face
		output.neighbors.at(i).proximals=vector<unsigned int>(20);
    	//* Compute the real position of the vertices
		mesh_msgs::TriangleIndices cur = output.mesh.mesh_geometry.faces.at(i); // store the current face
		// Vertices transformation
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
						// Transform the point to the camera reference fame
						temp1=transform2*temp2;
						// Save the results into temporary variables
						float temp01,temp02,temp03;
						temp01=temp1.getX();
						temp02=temp1.getY();
						temp03=temp1.getZ();
						// Switch the coordinates to conform to the camera frame convention
						temp1.setX(temp02);
						temp1.setY(temp03);
						temp1.setZ(temp01);
						// Transform the point into the image reference frame (this put the origin in the top left corner)
						temp1=transform1*temp1;
						//DEBUGGING CODE
						//x_array[j]=param.image.width-temp1.getX()/(temp1.getZ());
						//y_array[j]=param.image.height-temp1.getY()/(temp1.getZ());
						//ROS_INFO("%f %f %f %f %f",x_array[j],y_array[j],temp01,temp02,temp03);
						// Store the transformed point in the correct place 
						vertices[cur.vertex_indices[j]].transformed.x=temp1.getX();
						vertices[cur.vertex_indices[j]].transformed.y=temp1.getY();
						vertices[cur.vertex_indices[j]].transformed.z=temp1.getZ();
						vertices[cur.vertex_indices[j]].check=true; // Set the transformation check to true
						mutex->unlock(); // Unlock the mutex so that other can modify other points if they need
						//ROS_INFO("%f %f %f",temp1.getX(),temp1.getY(),temp1.getZ());
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
			// Convert the coordinates in pixels coordinate
			 x_array[j]=param.image.width-temp1.getX()/(temp1.getZ());
			 y_array[j]=param.image.height-temp1.getY()/(temp1.getZ());
		}

		// Operation over the faces
		
		// First we need to sort the array elements from top to bottom and from left to right and the coordinates need to be rounded to fit with a pixel
		// Define some variables for the calculations
		
		meanx=0.0;
		meany=0.0;
		// Loop over the three elements to find the minimum and the maximum over the y(in case of draw select order from left to right)
		float order[3]={y_array[0]*param.image.width+x_array[0],y_array[1]*param.image.width+x_array[1],y_array[2]*param.image.width+x_array[2]};
		for(int k=0;k<3;k++){
			if(min_<order[k]){
				min_=order[k];
				indmin=k;
			}
			if(max_>order[k]){
				max_=order[k];
				indmax=k;
			}
			meanx+=x_array[k];
			meany+=y_array[k];
		}
		meanx/=3;
		meany/=3;
		// Now indmin and indmax should correspond respectively to the points one and three
		// now we want to find the index for point two
		//ROS_INFO("%d %d",indmin, indmax);
		for(int k=0;k<3;k++){
			indmed=3-indmin-indmax;
			
			//	Round the pixel indices to correspond to
			if(x_array[k]<meanx){
				xarray[k]=floor(x_array[k]);
			}else{
				xarray[k]=ceil(x_array[k]);
			}

			if(y_array[k]<meany){
				yarray[k]=floor(y_array[k]);
			}else{
				yarray[k]=ceil(y_array[k]);
			}
		}
		float m3=0;
		int temp=0;
		if(xarray[indmax]-xarray[indmin]==0){
			m3= x_array[indmax];
			temp=1;
		}else{
			m3=float(yarray[indmax]-yarray[indmin])/float(xarray[indmax]-xarray[indmin]); // find the slope of the other line
			temp=0;
		}
		float features[9]={0}; // features of the face
		if(abs(yarray[indmax]-yarray[indmin])>threshold){
			// Compute the pixels inside the face
			list<int> pixels=list<int>(1); // initialize the pixel list
			int shift=addPixels(pixels,yarray,xarray,indmax,indmed,m3,temp,indmin,param.image.width,0); // compute the pixels for the upper triangle
			shift=addPixels(pixels,yarray,xarray,indmed,indmin,m3,temp,indmin,param.image.width,1); // compute the pixels for the lower triangle
			//ROS_INFO("%d",pixels.size());

			// Extract the rgb information for each pixel 
			list<RGB> colors=list<RGB>(0); // initialization of the list of colors for the face
			colors.front()=image.ptr<RGB>(pixels.front()%param.image.width)[pixels.front()/param.image.width];
			
			// Extract the features from these information
			for(int p : pixels){
				colors.insert(colors.end(),image.ptr<RGB>(pixels.front()%param.image.width)[pixels.front()/param.image.width]); // add the color to the list of colors of the face pixels
			}
			//ROS_INFO("%d %d %d", colors.front().red,colors.front().blue,colors.front().green);
			// Put the features in the output
			
			features_extraction( colors, features); // compute the features
		}
		// Initialize the feature portion for the face in the output
		output.face_features.at(i)=Features();
		// Input the features in the output
		for(int l=0;l<9;l++){
			//ROS_INFO("%f",features[l]);
			output.face_features.at(i).features.at(l)=features[l];//features[l];
		}
  	} 
	  return;
}

// Master thread function
void master_thread(traversability_mesh_package::Data param){
  
	// Transformation from LiDAR to camera
  /* ..Can be hardcoded or passed as a parameter(in the final version it will be like this), the most important feature is that is constant.. */
	geometry_msgs::Transform T_lc;
  T_lc.translation.x=transform_lc[0]; // translation of the LiDar from the camera along the x-axis
  T_lc.translation.y=transform_lc[1]; // translation of the LiDar from the camera along the y-axis
	T_lc.translation.z=transform_lc[2]; // translation of the LiDar from the camera along the z-axis
  geometry_msgs::Quaternion const_rot; // retrieve the quaternion of the LiDAR reference system w.r.t. the trunk reference system
  
  const_rot.x=transform_lc[3]; // rotation of the LiDar along the x-axis
  const_rot.y=transform_lc[4]; // rotation of the LiDar along the y-axis
  const_rot.z=transform_lc[5]; // rotation of the LiDar along the z-axis
  const_rot.w=transform_lc[6]; // rotation of the LiDar along the w-axis
 	
	//ROS_INFO("%f %f %f %f %f %f %f",transform_lc[0],transform_lc[1],transform_lc[2],transform_lc[3],transform_lc[4],transform_lc[5],transform_lc[6]);
	//ROS_INFO("%f %f %f %f %f %f %f",P[0],P[1],P[2],P[3],P[4],P[5],P[6]);


	 T_lc.rotation=const_rot;

	// Transformation from camera to pixel-space
  tf::Matrix3x3 t_cp_temp_rot=tf::Matrix3x3(P[0],P[1],P[2],P[4],P[5],P[6],P[8],P[9],P[10]); // reorganization of the terms composing the rotation in a 3x3 matrix
	tf::Vector3 t_cp_temp_transl; // declaration of the translation vector
	t_cp_temp_transl.setValue(P[3],P[7],P[11]); // initialization of the translation vector with the remaining terms
	tf::Transform T_cp=tf::Transform(t_cp_temp_rot,t_cp_temp_transl); // generation of the 4x4 transformation matrix

  // Compute the final transformation matrix from LiDAR to world (T[trunk to world]*T[LiDAR to trunk])
  tf::Transform T_w, res; // Prepare tf::Transform to perform the calculations
  tf::transformMsgToTF(T_lc,T_w); // Transform the lidar to camera in a transform
	res.mult(T_cp,T_w); // Multiply the two transforms and store them in the resulting final transform (This will be used to compute the lidar point position with respect to the pixel of the camera)
	//res=T_w;

  
  
	//We declare same variables to make the computation easier

	int n_vertices = param.mesh.mesh_geometry.vertices.size(); //number of vertices
	int n_faces = param.mesh.mesh_geometry.faces.size(); // number of faces
  int p_index; // index of the vertex we are considering at each instant
	Vertices vertices[n_vertices]; // Initialization of the vertices array

  //* Initialize the output
  // Generates arrays of appropriate dimension
	
	output.header=param.mesh.header;
	output.mesh=param.mesh;
	output.areas=vector<double>(n_faces);;
	output.slopes=vector<double>(n_faces);
	output.normals=vector<geometry_msgs::Point>(n_faces);
	output.baricenters=vector<geometry_msgs::Point>(n_faces);
	output.neighbors=vector<traversability_mesh_package::Proximals>(n_faces);
	output.face_features=vector<traversability_mesh_package::Features>(n_faces);


	//* Initialize the vertices array and give compute all faces neighboring to each vertex
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
				//vertices[p_index].neighbors[vertices[p_index].index]=i; // say that the first neighboring face is the one at which the face has been initialized
				vertices[p_index].neighbors.insert(vertices[p_index].neighbors.end(),i);
				vertices[p_index].index++; // Increment the writing index
			}
		}
	}
	
	// Image conversion to opencv to access rgb data
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(param.image,param.image.encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  //* Divide the faces among the number of servers
	boost::mutex mutex;
	boost::thread s_t[n_server];
	
  // compute how many faces should go to each server
	unsigned int portion = n_faces/n_server; // how many faces the server should handle
  	for(int i=0; i<n_server;i++)
			s_t[i]=boost::thread(server_thread,i,portion,vertices,param,cv_ptr->image,P[0],T_cp,T_w, &mutex);

	for(int  i=0;i<n_server;i++)
			s_t[i].join();

	
  //* Send the feature message 
	output_pub.publish(output);
	return;

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
  ros::init(argc, argv, "visual_extractor");

  //* Handle for the node
  ros::NodeHandle n;
  n.getParam("camera_parameters",P);
  n.getParam("transform_camera_lidar",transform_lc);
	//* Subscribe to the coordinated data topic
  ros::Subscriber input_sub = n.subscribe("base_coord", 10, callback);
	//* Advertize the coordinated data
  output_pub = n.advertise<GeoFeature>("/VisFeatures", 1000);

	ros::spin();

  //exit
  return 0;
}
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lvr_ros/ReconstructAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <boost/bind.hpp>


//Variable definition
sensor_msgs::PointCloud2 cloud; // global Point cloud variable
mesh_msgs::MeshGeometryStamped mesh; // global mesh variable
bool active=false;

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const lvr_ros::ReconstructResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  mesh=result->mesh;
  active=false;
}


// Called once when the goal becomes active
void activeCb()
{
  // The action server is active
  ROS_INFO("Goal just went active");
  active=true;
}


// Function to extract the Point Cloud
void PointCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  
  // Assignment of the received message properties to the global variable
  cloud.header=msg->header;
  cloud.height=msg->height;
  cloud.width=msg->width;
  cloud.fields=msg->fields;
  cloud.is_bigendian=msg->is_bigendian;
  cloud.point_step=msg->point_step;
  cloud.row_step=msg->row_step;
  cloud.data=msg->data;
  cloud.is_dense=msg->is_dense;
  
}


// Main definition
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mesh_creator");
  ROS_INFO("HI");
  ros::NodeHandle n;
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<lvr_ros::ReconstructAction> ac("reconstruction", true);
  // wait for the action server to start
  ROS_INFO("Start waiting for server");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, ready to send goal.");
  
  // Subscribe to the Point Cloud topic of the LiDAR
  ros::Subscriber sub = n.subscribe("vlp16_points", 1000, PointCallback);
  ROS_INFO("Subscribed to the point cloud");
  // Publish the mesh
  ros::Publisher mesh_pub = n.advertise<mesh_msgs::MeshGeometryStamped>("meshes", 1000);
  ROS_INFO("Waiting for action server to start.");
  
 

  
 
  //Loop to publish the mesh continuously
  //ros::Rate rate(200);
  //while(ros::ok())
  //{   
	 
	if(!active){ // if the action server is not active, activate it
       // send a goal to the action
       lvr_ros::ReconstructGoal goal; // define the goal
       goal.cloud = cloud; // assign the point cloud to the goal
       ac.sendGoal(goal, &doneCb, &activeCb); // send the goal to the action server
       //wait for the action to return
    }
       //Wait for the next loop
       //loop_rate.sleep();               
  //}

  //exit
  return 0;
}

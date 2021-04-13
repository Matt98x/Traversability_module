#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lvr_ros/ReconstructAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>



// Main definition
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mesh_creator");


  ros::NodeHandle n;
  
  //* Subscribe to the Point Cloud topic of the LiDAR
/*
  ros::Subscriber point_sub = n.subscribe("pointcloud", 100, cloud_callback);
  ros::Subscriber pose_sub = n.subscribe("odom",100,odom_callback);
*/
  point_pub = n.advertise<sensor_msgs::PointCloud2>("checkpoint", 1000);
	image_pub = n.advertise<sensor_msgs::Image>("image", 1000);
	odom_pub = n.advertise<nav_msgs::Odometry>("pose", 1000);

  message_filters::Subscriber<Image> image_sub(n, "mesh", 1);
  message_filters::Subscriber<PointCloud2> points_sub(n, "pointcloud", 1);
	message_filters::Subscriber<Odometry> odom_sub(n, "odom", 1);
  TimeSynchronizer<Image, PointCloud2, Odometry> sync(image_sub, points_sub,odom_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

 
  //* Loop to publish the mesh continuously
  ros::Rate rate(5);
  while(ros::ok())
  {     
     point_pub.publish(cloud);
		 odom_pub.publish(odom);
		 image_pub.publish(image);

     //Wait for the next loop
     ros::spinOnce();
     rate.sleep();              
  }

  
  //exit
  return 0;
}                                                                                              

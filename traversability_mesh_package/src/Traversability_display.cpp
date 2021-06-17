//* Necessary libraries
#include <ros/ros.h> // roslibrary
// Input mesh library
#include <mesh_msgs/TriangleMeshStamped.h>
// Output marker library
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>

ros::Publisher trav_pub;
ros::Subscriber trav_sub;

//* Callback function for the synchronizer
void callback(const mesh_msgs::TriangleMeshStamped::Ptr& msg)
{   mesh_msgs::TriangleMeshStamped msgs=*msg;
	//* Initialize the output
    visualization_msgs::MarkerArray output=visualization_msgs::MarkerArray();
    for (int i=0;i<msgs.mesh.triangles.size();i++){
        //* Initialize the marker for the face
        visualization_msgs::Marker temp=visualization_msgs::Marker();
        temp.header=msgs.header;
        temp.ns="markers";
        temp.id=i;
        temp.type=11;
        temp.action=visualization_msgs::Marker::ADD;
        temp.pose.position.x = 0;
        temp.pose.position.y = 0;
        temp.pose.position.z = 0;
        temp.pose.orientation.x = 0.0;
        temp.pose.orientation.y = 0.0;
        temp.pose.orientation.z = 0.0;
        temp.pose.orientation.w = 1.0;
        temp.scale.x = 1;
        temp.scale.y = 1;
        temp.scale.z = 1;
        temp.color.a = msgs.mesh.face_materials[i].color.a; // Don't forget to set the alpha!
        temp.color.r = msgs.mesh.face_materials[i].color.r;
        temp.color.g = msgs.mesh.face_materials[i].color.g;
        temp.color.b = msgs.mesh.face_materials[i].color.b;
        for(int k=0;k<3;k++){
            geometry_msgs::Point vertex=geometry_msgs::Point();
            vertex.x=msgs.mesh.vertices[msgs.mesh.triangles[i].vertex_indices[k]].x;
            vertex.y=msgs.mesh.vertices[msgs.mesh.triangles[i].vertex_indices[k]].y;
            vertex.z=msgs.mesh.vertices[msgs.mesh.triangles[i].vertex_indices[k]].z;
            temp.points.insert(temp.points.end(),vertex);
        }
        //* Publish the output
        output.markers.push_back(temp);
    }
    
    trav_pub.publish(output);
}

// Main definition
int main (int argc, char **argv)
{
	//* Node initiation
  ros::init(argc, argv, "mesh_display");

  //* Handle for the node
  ros::NodeHandle n;

	//* Subscribe to the pointcloud
  trav_sub = n.subscribe("test_msg", 1000, callback);
  //* Advertize the coordinated data
  trav_pub = n.advertise<visualization_msgs::MarkerArray>("display", 1000);
    //* Loop
    ros::spin();
}

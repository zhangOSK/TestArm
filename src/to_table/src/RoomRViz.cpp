#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "MsgUtils.h"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "RoomRViz");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Rate r(1);

  while (ros::ok())
  {

    Vector3d pos_room(1.215, -0.51, 0.0); 
    Vector3d pos_stairs(1.07, -1.24, 0.0);    
    Quaterniond q(1.0,0.0,0.0,0.0);

    Vector3d color(0.6,0.6,0.6);    

    std::string mesh_room_path = "package://to_table/mesh/Room_gerard_bauzil_without_floor.stl";

    visualization_msgs::Marker marker_room = Eigen2MeshMarker(pos_room,q,
      "room_shape", 0, 1.0, color, mesh_room_path);

    // Publish the marker
    marker_pub.publish(marker_room);

    // std::string mesh_stairs_path = "package://to_table/mesh/Stairs.stl";

    // visualization_msgs::Marker marker_stairs = Eigen2MeshMarker(pos_stairs,q,
    //   "stairs_shape", 0, 0.001, color, mesh_stairs_path);

    // // Publish the marker
    // marker_pub.publish(marker_stairs);

    r.sleep();
  }

  return 0;
}

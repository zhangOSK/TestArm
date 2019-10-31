#include <ros/ros.h>
#include <math.h>
#include "MsgUtils.h"

class SubscribeAndPublishRviz
{
private:
  ros::NodeHandle n; 
  ros::Publisher pub_marker;
  ros::Subscriber sub;
  int count = 0;

public:
  void callback(const tf2_msgs::TFMessage::ConstPtr& msg)
  {
    Vector3d color_marker_robot(0.0,1.0,0.0);
    Vector3d color_marker_table(1.0,0.0,0.0); 
    Vector3d color_table(1.0,1.0,1.0);    
    std::string mesh_table_path = "package://to_table/mesh/Table.stl";

    if (msg->transforms[0].child_frame_id == "Table") 
    {
      // Marker for the position.
      Vector3d origin_mocap(0.0,0.0,0.0);

      Vector3d table_mocap(msg->transforms[0].transform.translation.x,
        msg->transforms[0].transform.translation.y,
        msg->transforms[0].transform.translation.z);        
      Translation<double,3> trT_mocap(table_mocap);

      Quaterniond qT(msg->transforms[0].transform.rotation.w,
        msg->transforms[0].transform.rotation.x,
        msg->transforms[0].transform.rotation.y,
        msg->transforms[0].transform.rotation.z);
      AngleAxisd rotT(qT);

      Vector3d pos_centerT(0.59, 0.385, -0.02);

      Translation<double,3> tr_centerT(pos_centerT);
      Vector3d pos = trT_mocap*rotT*tr_centerT*origin_mocap;

      visualization_msgs::Marker marker_table_pose = Eigen2MeshMarker(pos,qT,
        "table_shape", 0, 1.0, color_table,mesh_table_path);

      pub_marker.publish(marker_table_pose);

	    // Marker for trajectory.
      // visualization_msgs::Marker marker_table_traj = TFMessage2TrajectoryMarker(msg, 
      //  "marker_table",count, 0.02, color_marker_table, 40);

      // pub_marker.publish(marker_table_traj);

      //count += 1;
	  }

    if (msg->transforms[0].child_frame_id == "Talos_Torso") 
    {
	    // Marker for trajectory.
      visualization_msgs::Marker marker_robot_traj = TFMessage2TrajectoryMarker(msg, 
        "marker_robot",count, 0.02, color_marker_robot, 20);

      pub_marker.publish(marker_robot_traj);
      count += 1;
    }
  }

  SubscribeAndPublishRviz()
  {
  	pub_marker = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    sub = n.subscribe("tf", 1000, &SubscribeAndPublishRviz::callback, this);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PositionFilterRViz");

  SubscribeAndPublishRviz SPObject;

  ros::spin();

  return 0;
}

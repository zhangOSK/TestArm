#ifndef MSGTILS_H_INCLUDED
#define MSGUTILS_H_INCLUDED

#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/Marker.h>
#include "Utils.h"

geometry_msgs::TransformStamped LinkStates2TransformStamped(const gazebo_msgs::LinkStates::ConstPtr& input_msg, int i);

geometry_msgs::TransformStamped Eigen2TransformStamped(Vector3d pos, Quaterniond q);

geometry_msgs::TransformStamped ModelStates2TransformStamped(const gazebo_msgs::ModelStates::ConstPtr& input_msg, int i);

geometry_msgs::TransformStamped TFMessage2TransformStamped(const tf2_msgs::TFMessage::ConstPtr& input_msg);

visualization_msgs::Marker Eigen2MeshMarker(Vector3d pos, Quaterniond q,
    std::string namespace_name, int id, float scale, Vector3d color, std::string mesh_path);

visualization_msgs::Marker TFMessage2TrajectoryMarker(const tf2_msgs::TFMessage::ConstPtr& input_msg, 
    std::string namespace_name, int id, float scale, Vector3d color, int duration);

visualization_msgs::Marker Eigen2TrajectoryMarker(Vector3d pos,std::string namespace_name,
	int id, float scale, Vector3d color, int duration);

#endif // MSGUTILS_H_INCLUDED
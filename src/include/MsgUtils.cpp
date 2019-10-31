#include "MsgUtils.h"

geometry_msgs::TransformStamped LinkStates2TransformStamped(const gazebo_msgs::LinkStates::ConstPtr& input_msg, int i)
{
	geometry_msgs::TransformStamped output_msg;

    output_msg.transform.translation.x = input_msg->pose[i].position.x;
    output_msg.transform.translation.y = input_msg->pose[i].position.y;
    output_msg.transform.translation.z = input_msg->pose[i].position.z;
    output_msg.transform.rotation.x = nullFunction(input_msg->pose[i].orientation.x);
    output_msg.transform.rotation.y = nullFunction(input_msg->pose[i].orientation.y);
    output_msg.transform.rotation.z = nullFunction(input_msg->pose[i].orientation.z);
    output_msg.transform.rotation.w = nullFunction(input_msg->pose[i].orientation.w);

    output_msg.header.stamp = ros::Time::now();

    return output_msg;
}

geometry_msgs::TransformStamped Eigen2TransformStamped(Vector3d pos, Quaterniond q)
{
	geometry_msgs::TransformStamped output_msg;

    output_msg.transform.translation.x = pos(0);
    output_msg.transform.translation.y = pos(1);
    output_msg.transform.translation.z = pos(2);
    output_msg.transform.rotation.x = nullFunction(q.x());
    output_msg.transform.rotation.y = nullFunction(q.y());
    output_msg.transform.rotation.z = nullFunction(q.z());
    output_msg.transform.rotation.w = nullFunction(q.w());

    output_msg.header.stamp = ros::Time::now();

    return output_msg;
}

geometry_msgs::TransformStamped ModelStates2TransformStamped(const gazebo_msgs::ModelStates::ConstPtr& input_msg, int i)
{
    geometry_msgs::TransformStamped output_msg;

    output_msg.transform.translation.x = input_msg->pose[i].position.x;
    output_msg.transform.translation.y = input_msg->pose[i].position.y;
    output_msg.transform.translation.z = input_msg->pose[i].position.z;
    output_msg.transform.rotation.x = nullFunction(input_msg->pose[i].orientation.x);
    output_msg.transform.rotation.y = nullFunction(input_msg->pose[i].orientation.y);
    output_msg.transform.rotation.z = nullFunction(input_msg->pose[i].orientation.z);
    output_msg.transform.rotation.w = nullFunction(input_msg->pose[i].orientation.w);

    output_msg.header.stamp = ros::Time::now();

    return output_msg;    
}

geometry_msgs::TransformStamped TFMessage2TransformStamped(const tf2_msgs::TFMessage::ConstPtr& input_msg)
{
    geometry_msgs::TransformStamped output_msg;

    output_msg.transform.translation.x = input_msg->transforms[0].transform.translation.x;
    output_msg.transform.translation.y = input_msg->transforms[0].transform.translation.y;
    output_msg.transform.translation.z = input_msg->transforms[0].transform.translation.z;
    output_msg.transform.rotation.x = nullFunction(input_msg->transforms[0].transform.rotation.x);
    output_msg.transform.rotation.y = nullFunction(input_msg->transforms[0].transform.rotation.y);
    output_msg.transform.rotation.z = nullFunction(input_msg->transforms[0].transform.rotation.z);
    output_msg.transform.rotation.w = nullFunction(input_msg->transforms[0].transform.rotation.w);
      
    output_msg.header.stamp = ros::Time::now();

    return output_msg;  
}

visualization_msgs::Marker Eigen2MeshMarker(Vector3d pos, Quaterniond q,
    std::string namespace_name, int id, float scale, Vector3d color, std::string mesh_path)
{
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/Qualisys";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = namespace_name;
    marker.id = id;

    // Set the marker type. 
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos(0);
    marker.pose.position.y = pos(1);
    marker.pose.position.z = pos(2);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker.mesh_resource = mesh_path;
    
    return marker;  
}

visualization_msgs::Marker TFMessage2TrajectoryMarker(const tf2_msgs::TFMessage::ConstPtr& input_msg, 
    std::string namespace_name, int id, float scale, Vector3d color, int duration)
{
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/Qualisys";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = namespace_name;
    marker.id = id;

    // Set the marker type. 
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = input_msg->transforms[0].transform.translation.x;
    marker.pose.position.y = input_msg->transforms[0].transform.translation.y;
    marker.pose.position.z = input_msg->transforms[0].transform.translation.z;
    marker.pose.orientation.x = input_msg->transforms[0].transform.rotation.x;
    marker.pose.orientation.y = input_msg->transforms[0].transform.rotation.y;
    marker.pose.orientation.z = input_msg->transforms[0].transform.rotation.z;
    marker.pose.orientation.w = input_msg->transforms[0].transform.rotation.w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(duration);
    
    return marker;  
}

visualization_msgs::Marker Eigen2TrajectoryMarker(Vector3d pos,std::string namespace_name,
    int id, float scale, Vector3d color, int duration)
{
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/Qualisys";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = namespace_name;
    marker.id = id;

    // Set the marker type. 
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos(0);
    marker.pose.position.y = pos(1);
    marker.pose.position.z = pos(2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = color(0);
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(duration);
    
    return marker;      
}
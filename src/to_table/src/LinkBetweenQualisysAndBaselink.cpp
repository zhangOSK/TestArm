#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <Eigen/Geometry>

using namespace Eigen;

void callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  	static tf::TransformBroadcaster br;

	if (msg->transforms[0].child_frame_id == "Talos_Torso") 
	{
		if (!std::isnan(msg->transforms[0].transform.translation.x))
		{
		  	tf::Transform position_base_link;

		  	Vector3d origin_mocap(0.0,0.0,0.0);

		  	Vector3d pos_robot_mocap(msg->transforms[0].transform.translation.x,
	        	msg->transforms[0].transform.translation.y,
	        	msg->transforms[0].transform.translation.z);
	        Quaterniond qR(msg->transforms[0].transform.rotation.w,
	        	msg->transforms[0].transform.rotation.x,
	        	msg->transforms[0].transform.rotation.y,
	        	msg->transforms[0].transform.rotation.z);
	        Translation<double,3> tr_robot_mocap(pos_robot_mocap);  

		  	Vector3d pos_center_robot(-0.07, 0, -0.11);
		  	Translation<double,3> tr_center_robot(pos_center_robot); 
		  	AngleAxisd rotR(qR);

		  	Vector3d pos = tr_robot_mocap*qR*tr_center_robot*origin_mocap;

	  		position_base_link.setOrigin(tf::Vector3(pos(0),pos(1),pos(2)));
	  		position_base_link.setRotation(tf::Quaternion(qR.x(),qR.y(),qR.z(),qR.w()));

			br.sendTransform(tf::StampedTransform(position_base_link, ros::Time::now(), "Qualisys", "base_link"));		
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "LinkBetweenQualisysAndBaselink");

	ros::NodeHandle n;
	ros::Subscriber sub;
	sub = n.subscribe("/tf", 1000, &callback);

	ros::spin();
	
	return 0;
};
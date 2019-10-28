#include <ros/ros.h>
#include <math.h>
#include "MsgUtils.h"

class SubscribeAndPublishSim
{
private:
  ros::NodeHandle n; 
  ros::Publisher pub_right_hand_sim;
  ros::Publisher pub_left_hand_sim;
  ros::Publisher pub_torso_sim;  
  ros::Subscriber sub;

public:

  void callback(const gazebo_msgs::LinkStates::ConstPtr& msg)
  {
    for (int i=0; i<msg->name.size(); i++)
    { 
      if (msg->name[i] == "talos::arm_right_7_link") 
      {
        geometry_msgs::TransformStamped position_rigth_hand_msg = LinkStates2TransformStamped(msg,i);
        pub_right_hand_sim.publish(position_rigth_hand_msg);
      }

      if (msg->name[i] == "talos::arm_left_7_link") 
      {
        geometry_msgs::TransformStamped position_left_hand_msg = LinkStates2TransformStamped(msg,i);      
        pub_left_hand_sim.publish(position_left_hand_msg);
      }

      if (msg->name[i] == "talos::torso_1_link") 
      {
        geometry_msgs::TransformStamped position_torso_msg = LinkStates2TransformStamped(msg,i);      
        pub_torso_sim.publish(position_torso_msg);
      }

    }

  }

  SubscribeAndPublishSim()
  {
    pub_right_hand_sim = n.advertise<geometry_msgs::TransformStamped>("position_right_hand_sim", 1000);
    pub_left_hand_sim = n.advertise<geometry_msgs::TransformStamped>("position_left_hand_sim", 1000);
    pub_torso_sim = n.advertise<geometry_msgs::TransformStamped>("position_torso_sim", 1000);
    sub = n.subscribe("/gazebo/link_states", 1000, &SubscribeAndPublishSim::callback, this);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PositionHandsSim");

  SubscribeAndPublishSim PMObject;

  ros::spin();

  return 0;
}

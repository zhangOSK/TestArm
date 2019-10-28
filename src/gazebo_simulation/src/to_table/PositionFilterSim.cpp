#include <ros/ros.h>
#include <math.h>
#include "MsgUtils.h"



class SubscribeAndPublishSim
{
private:
  ros::NodeHandle n; 
  ros::Publisher pub_table1_sim;
  ros::Publisher pub_table2_sim;
  ros::Publisher pub_robot_sim;
  ros::Publisher pub_talos_sim;
  ros::Subscriber sub;

public:

  void callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    double l; // distance between the robot_torso body and the table edge
    n.getParam("dist_table_robot", l);

    Eigen::Vector3d origin_mocap(0.0,0.0,0.0);

    Eigen::Vector3d pos1_table(-l-0.01,0.385,0.0);
    Eigen::Vector3d pos2_table(1.199+l,0.385,0.0);

    Eigen::Translation<double,3> tr_table_pos1(pos1_table);
    Eigen::Translation<double,3> tr_table_pos2(pos2_table);

    for (int i=0; i<msg->name.size(); i++)
    {
      if (msg->name[i] == "my_table") 
      {
        Eigen::Quaterniond q1(nullFunction(msg->pose[i].orientation.w),
          nullFunction(msg->pose[i].orientation.x),
          nullFunction(msg->pose[i].orientation.y),
          nullFunction(msg->pose[i].orientation.z));
        Eigen::AngleAxisd rotation(q1);

        Eigen::Vector3d origin_table_mocap(msg->pose[i].position.x,msg->pose[i].position.y,msg->pose[i].position.z);        
        Eigen::Translation<double,3> tr_mocap_table(origin_table_mocap);

        Eigen::Vector3d pos1_table_mocap = tr_mocap_table*rotation*tr_table_pos1*origin_mocap;
        Eigen::Vector3d pos2_table_mocap = tr_mocap_table*rotation*tr_table_pos2*origin_mocap;

        Eigen::Quaterniond qPI(cos(M_PI/2),0.0,0.0,sin(M_PI/2));
        Eigen::AngleAxisd rotationPI(qPI);
        Eigen::Quaterniond q2 = q1*qPI;

        geometry_msgs::TransformStamped position_table1_msg = Eigen2TransformStamped(pos1_table_mocap, q1);
        geometry_msgs::TransformStamped position_table2_msg = Eigen2TransformStamped(pos2_table_mocap, q2);

        pub_table1_sim.publish(position_table1_msg);
        pub_table2_sim.publish(position_table2_msg);
      }

      if (msg->name[i] == "my_robot") 
      {
        geometry_msgs::TransformStamped position_robot_msg = ModelStates2TransformStamped(msg, i);
        pub_robot_sim.publish(position_robot_msg);
      }

      if (msg->name[i] == "talos") 
      {
        geometry_msgs::TransformStamped position_talos_msg = ModelStates2TransformStamped(msg, i);
        pub_talos_sim.publish(position_talos_msg);
      }
    }
    
  }


  SubscribeAndPublishSim()
  {
    pub_table1_sim = n.advertise<geometry_msgs::TransformStamped>("position_table1_sim", 1000);
    pub_table2_sim = n.advertise<geometry_msgs::TransformStamped>("position_table2_sim", 1000);
    pub_robot_sim = n.advertise<geometry_msgs::TransformStamped>("position_robot_sim", 1000);
    pub_talos_sim = n.advertise<geometry_msgs::TransformStamped>("position_talos_sim", 1000);
    sub = n.subscribe("/gazebo/model_states", 1000, &SubscribeAndPublishSim::callback, this);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PositionFilterSim");

  SubscribeAndPublishSim PMObject;

  ros::spin();

  return 0;
}

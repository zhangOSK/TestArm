#include <ros/ros.h>
#include <math.h>
#include "MsgUtils.h"

class SubscribeAndPublishSim
{
private:
  ros::NodeHandle n; 
  ros::Publisher pub_table1_sim;
  ros::Publisher pub_table2_sim;
  ros::Subscriber sub;

public:

  void callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
  {
    Vector3d origin_mocap(0.0,0.0,0.0);

    Vector3d pos1_table(-0.01,0.385,0.0);
    Vector3d pos2_table(1.199,0.385,0.0);

    Translation<double,3> tr_table_pos1(pos1_table);
    Translation<double,3> tr_table_pos2(pos2_table);

    for (int i=0; i<msg->name.size(); i++)
    {
      if (msg->name[i] == "my_table") 
      {
        Quaterniond q1(msg->pose[i].orientation.w,msg->pose[i].orientation.x,
          msg->pose[i].orientation.y,msg->pose[i].orientation.z);
        AngleAxisd rotation(q1);

        Vector3d origin_table_mocap(msg->pose[i].position.x,msg->pose[i].position.y,msg->pose[i].position.z+0.77);        
        Translation<double,3> tr_mocap_table(origin_table_mocap);

        Vector3d pos1_table_mocap = tr_mocap_table*rotation*tr_table_pos1*origin_mocap;
        Vector3d pos2_table_mocap = tr_mocap_table*rotation*tr_table_pos2*origin_mocap;

        Quaterniond qPI(cos(M_PI/2),0.0,0.0,sin(M_PI/2));
        AngleAxisd rotationPI(qPI);
        Quaterniond q2 = q1*qPI;

        geometry_msgs::TransformStamped position_table1_msg = Eigen2TransformStamped(pos1_table_mocap, q1);
        geometry_msgs::TransformStamped position_table2_msg = Eigen2TransformStamped(pos2_table_mocap, q2);

        pub_table1_sim.publish(position_table1_msg);
        pub_table2_sim.publish(position_table2_msg);
      }
    }
  }

  SubscribeAndPublishSim()
  {
    pub_table1_sim = n.advertise<geometry_msgs::TransformStamped>("position_table1_sim", 1000);
    pub_table2_sim = n.advertise<geometry_msgs::TransformStamped>("position_table2_sim", 1000);
    sub = n.subscribe("/gazebo/model_states", 1000, &SubscribeAndPublishSim::callback, this);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PositionTableSim");

  SubscribeAndPublishSim PMObject;

  ros::spin();

  return 0;
}

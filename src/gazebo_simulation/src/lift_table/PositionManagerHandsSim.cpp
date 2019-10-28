#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h> 
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include "Utils.h"

class MultipleSubscribeAndPublishSim
{
private:
  ros::NodeHandle n; 
  ros::Publisher pub_rh_sim;
  ros::Publisher pub_lh_sim;

  int count = 0;

  message_filters::Subscriber<geometry_msgs::TransformStamped> table1_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> table2_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> rh_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> lh_sub_sim;  
  message_filters::Subscriber<geometry_msgs::TransformStamped> torso_sub_sim;

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped,
  geometry_msgs::TransformStamped,geometry_msgs::TransformStamped,
  geometry_msgs::TransformStamped,geometry_msgs::TransformStamped> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

public:

  void handCmd(Vector3d posT,Quaterniond qT,Vector3d posRH,Vector3d posLH,Quaterniond qR)
  {
    geometry_msgs::PointStamped rh_msg;
    geometry_msgs::PointStamped lh_msg;

    Vector3d origin_mocap(0.0,0.0,0.0);

    Vector3d pos_rh_table(-0.14,-0.25,-0.03);
    Vector3d pos_lh_table(-0.14,0.25,-0.03);

    Translation<double,3> tr_table_rh(pos_rh_table);
    Translation<double,3> tr_table_lh(pos_lh_table);
    Translation<double,3> tr_mocap_rh(posRH);
    Translation<double,3> tr_mocap_lh(posLH);
    Translation<double,3> tr_mocap_table(posT);
    AngleAxisd rot_mocap_table(qT);
    AngleAxisd rot_robot_mocap(qR);

    Vector3d pos_rh_table_mocap = (tr_mocap_rh*rot_robot_mocap).inverse()*tr_mocap_table*rot_mocap_table*tr_table_rh*origin_mocap;
    Vector3d pos_lh_table_mocap = (tr_mocap_lh*rot_robot_mocap).inverse()*tr_mocap_table*rot_mocap_table*tr_table_lh*origin_mocap;  

    double theta_table = thetaFct(qT);
    double theta_robot = thetaFct(qR);

    rh_msg.point.x = pos_rh_table_mocap(0);
    rh_msg.point.y = pos_rh_table_mocap(1);   
    rh_msg.point.z = theta_table-theta_robot;

    lh_msg.point.x = pos_lh_table_mocap(0);
    lh_msg.point.y = pos_lh_table_mocap(1);
    lh_msg.point.z = theta_table-theta_robot;

    if (count%100 == 0)
    {
      ROS_INFO_STREAM("Table ##############");
      ROS_INFO_STREAM("x = " << posT(0) << ", y = " << posT(1) << ", z = " << posT(2));      
       
      ROS_INFO_STREAM("Right Hand ##########");
      ROS_INFO_STREAM("x = " << posRH(0) << ", y = " << posRH(1) << ", z = " << posRH(2));      

      ROS_INFO_STREAM("Left Hand ###########");
      ROS_INFO_STREAM("x = " << posLH(0) << ", y = " << posLH(1) << ", z = " << posLH(2));      

      ROS_INFO_STREAM("Delta Right Hand ####");
      ROS_INFO_STREAM("x = " << pos_rh_table_mocap(0) << ", y = " << pos_rh_table_mocap(1) << ", z = " << pos_rh_table_mocap(2));      
      
      ROS_INFO_STREAM("Delta Left Hand ######");
      ROS_INFO_STREAM("x = " << pos_lh_table_mocap(0) << ", y = " << pos_lh_table_mocap(1) << ", z = " << pos_lh_table_mocap(2));           
    
      ROS_INFO_STREAM("Delta Theta ######");
      ROS_INFO_STREAM("theta T = " << theta_table << ", theta R = " << theta_robot);           
    }

    count += 1;

    rh_msg.header.stamp = ros::Time::now();
    lh_msg.header.stamp = ros::Time::now();

    pub_rh_sim.publish(rh_msg); 
    pub_lh_sim.publish(lh_msg); 
  }

  void callback(const geometry_msgs::TransformStamped::ConstPtr& pos1,
    const geometry_msgs::TransformStamped::ConstPtr& pos2,
    const geometry_msgs::TransformStamped::ConstPtr& posRH,
    const geometry_msgs::TransformStamped::ConstPtr& posLH,
    const geometry_msgs::TransformStamped::ConstPtr& posR)
  {
    float delta_x;
    float delta_y;
    float delta_theta;
    Vector3d delta;

    Vector3d posT1(pos1->transform.translation.x,pos1->transform.translation.y,pos1->transform.translation.z);
    Vector3d posT2(pos2->transform.translation.x,pos2->transform.translation.y,pos2->transform.translation.z);
    Vector3d pos_right_hand(posRH->transform.translation.x,posRH->transform.translation.y,posRH->transform.translation.z);
    Vector3d pos_left_hand(posLH->transform.translation.x,posLH->transform.translation.y,posLH->transform.translation.z);
    Vector3d pos_torso(posR->transform.translation.x,posR->transform.translation.y,posR->transform.translation.z);
    
    Quaterniond qRob(posR->transform.rotation.w,posR->transform.rotation.x,
      posR->transform.rotation.y,posR->transform.rotation.z);

    float distance1 = distance(posT1,pos_torso);
    float distance2 = distance(posT2,pos_torso);   

    // Choose the closest position
    if (distance1 < distance2)
    { 
      //ROS_INFO_STREAM("Pos1 choisie! ----------------------------------------------------");
      Quaterniond qT1(pos1->transform.rotation.w,pos1->transform.rotation.x,
        pos1->transform.rotation.y,pos1->transform.rotation.z);             
      handCmd(posT1,qT1,pos_right_hand,pos_left_hand,qRob);

    } else {
      //ROS_INFO_STREAM("Pos2 choisie! ----------------------------------------------------"); 
      Quaterniond qT2(pos2->transform.rotation.w,pos2->transform.rotation.x,
        pos2->transform.rotation.y,pos2->transform.rotation.z);              
      handCmd(posT2,qT2,pos_right_hand,pos_left_hand,qRob);
    }
  }

  MultipleSubscribeAndPublishSim()
  {
    table1_sub_sim.subscribe(n,"position_table1_sim", 100);
    table2_sub_sim.subscribe(n,"position_table2_sim", 100);
    rh_sub_sim.subscribe(n,"position_right_hand_sim", 100);
    lh_sub_sim.subscribe(n,"position_left_hand_sim", 100);  
    torso_sub_sim.subscribe(n,"position_torso_sim", 100);      
    sync.reset(new Sync(MySyncPolicy(10), table1_sub_sim, table2_sub_sim, rh_sub_sim, lh_sub_sim, torso_sub_sim));      
    sync->registerCallback(boost::bind(&MultipleSubscribeAndPublishSim::callback, this, _1, _2, _3, _4, _5));

    pub_rh_sim = n.advertise<geometry_msgs::PointStamped>("cmd_right_hand_sim", 10);
    pub_lh_sim = n.advertise<geometry_msgs::PointStamped>("cmd_left_hand_sim", 10);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PositionManagerHandsSim");

  MultipleSubscribeAndPublishSim MSPObject;

  ros::spin();

  return 0;
}
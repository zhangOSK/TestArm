#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <math.h> 
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <gazebo_msgs/ModelState.h>
#include "Utils.h"


#include <string>
#include <boost/shared_ptr.hpp>
// ROS headers
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>

class MultipleSubscribeAndPublishSim
{
private:
  ros::NodeHandle n; 
  ros::Publisher pub_sim;

  message_filters::Subscriber<geometry_msgs::TransformStamped> table1_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> table2_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> robot_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> talos_sub_sim;

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped,geometry_msgs::TransformStamped,geometry_msgs::TransformStamped> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync; 

  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> arm_control_client;
  typedef boost::shared_ptr< arm_control_client>  arm_control_client_Ptr;

public:

  Eigen::Vector3d deltaFct(Eigen::Vector3d posT,Eigen::Quaterniond qT,Eigen::Vector3d posR,Eigen::Quaterniond qR)
  {
    Eigen::Vector3d angleT = qT.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector3d angleR = qR.toRotationMatrix().eulerAngles(0, 1, 2);

    Eigen::Vector3d delta(posT(0)-posR(0),posT(1)-posR(1),angleT(2)-angleR(2));

    return delta;
  }

  //--------------------------------------
  // Create a ROS action client to move TIAGo's arm
  void createArmClient(arm_control_client_Ptr& actionClient)
  {
    ROS_INFO("Creating action client to arm controller ...");

    ////actionClient.reset( new arm_control_client("/arm_controller/follow_joint_trajectory") );
    actionClient.reset( new arm_control_client("/left_arm_controller/follow_joint_trajectory") );
    
    int iterations = 0, max_iterations = 3;
    // Wait for arm controller action server to come up
    while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
    {
      ROS_DEBUG("Waiting for the arm_controller_action server to come up");
      ++iterations;
    }

    if ( iterations == max_iterations )
      throw std::runtime_error("Error in createArmClient: arm controller action server not available");
    
  }
  // Generates a simple trajectory with two waypoints to move TIAGo's arm 
  void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal)
  {
    // The joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("arm_left_1_joint");
    goal.trajectory.joint_names.push_back("arm_left_2_joint");
    goal.trajectory.joint_names.push_back("arm_left_3_joint");
    goal.trajectory.joint_names.push_back("arm_left_4_joint");
    goal.trajectory.joint_names.push_back("arm_left_5_joint");
    goal.trajectory.joint_names.push_back("arm_left_6_joint");
    goal.trajectory.joint_names.push_back("arm_left_7_joint");

    // Two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    //0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1

    // First trajectory point
    // Positions
    int index = 0;
    goal.trajectory.points[index].positions.resize(7);
    goal.trajectory.points[index].positions[0] = 0.0;
    goal.trajectory.points[index].positions[1] = 0.0;
    goal.trajectory.points[index].positions[2] = 0.0;
    goal.trajectory.points[index].positions[3] = -0.5;
    goal.trajectory.points[index].positions[4] = 0.0;
    goal.trajectory.points[index].positions[5] = -0.0;
    goal.trajectory.points[index].positions[6] = 0.1;
    // Velocities
    goal.trajectory.points[index].velocities.resize(7);
    for (int j = 0; j < 7; ++j)
    {
      goal.trajectory.points[index].velocities[j] = 1.0; //1.0
    }
    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(1);

    
    /*index += 1;
    goal.trajectory.points[index].positions.resize(7);
    goal.trajectory.points[index].positions[0] = 0.25;
    goal.trajectory.points[index].positions[1] = 0.17;
    goal.trajectory.points[index].positions[2] = 0.0;
    goal.trajectory.points[index].positions[3] = -0.5;
    goal.trajectory.points[index].positions[4] = 0.0;
    goal.trajectory.points[index].positions[5] = -0.0;
    goal.trajectory.points[index].positions[6] = 0.1;
    // Velocities
    goal.trajectory.points[index].velocities.resize(7);
    for (int j = 0; j < 7; ++j)
    {
      goal.trajectory.points[index].velocities[j] = 0.0;
    }
    // To be reached 2 second after starting along the trajectory
    goal.trajectory.points[index].time_from_start = ros::Duration(2);*/
    
  }

  void callback(const geometry_msgs::TransformStamped::ConstPtr& pos1,const geometry_msgs::TransformStamped::ConstPtr& pos2,const geometry_msgs::TransformStamped::ConstPtr& posR)
  {
    float delta_x;
    float delta_y;
    float delta_theta;
    Eigen::Vector3d delta;

    Eigen::Vector3d pos1_table_mocap(pos1->transform.translation.x,pos1->transform.translation.y,pos1->transform.translation.z);
    Eigen::Vector3d pos2_table_mocap(pos2->transform.translation.x,pos2->transform.translation.y,pos2->transform.translation.z);
    Eigen::Vector3d pos_robot_mocap(posR->transform.translation.x,posR->transform.translation.y,posR->transform.translation.z);

    Eigen::Quaterniond qR(posR->transform.rotation.w,posR->transform.rotation.x,
      posR->transform.rotation.y,posR->transform.rotation.z);
    Eigen::Quaterniond qT1(pos1->transform.rotation.w,pos1->transform.rotation.x,
      pos1->transform.rotation.y,pos1->transform.rotation.z);    
    Eigen::Quaterniond qT2(pos2->transform.rotation.w,pos2->transform.rotation.x,
      pos2->transform.rotation.y,pos2->transform.rotation.z);

    float distance1 = distance(pos1_table_mocap,pos_robot_mocap);
    float distance2 = distance(pos2_table_mocap,pos_robot_mocap);   

    // Choose the closest position
    if (distance1 < distance2)
    {   
      delta = deltaFct(pos1_table_mocap,qT1,pos_robot_mocap,qR);

    } else {      
      delta = deltaFct(pos2_table_mocap,qT2,pos_robot_mocap,qR);
    }

    delta_x = delta(0); 
    delta_y = delta(1); 
    delta_theta = delta(2);       

    // Send 0 command if the robot is closer enough to the position (linear velocity < 2 cm.s-1 and angular velocity < 3 deg.s-1)
    if (fabs(delta_x) < 0.05)
    { 
      delta_x = 0;
    }

    if (fabs(delta_y) < 0.05)
    {
      delta_y = 0;
    }

    if (fabs(delta_theta) < 0.08)
    {
      delta_theta = 0;
    }

    if (delta_x == 0 && delta_y == 0 && delta_theta == 0)
    {
      n.setParam("dist_table_robot", 0.48);

    }

    //Apply a threshold if needed (linear velocity < 10 cm.s-1 and angular velocity < 10 deg.s-1)
    if (fabs(delta_x) > 1 || fabs(delta_y) > 1)
    {
      if (fabs(delta_x) < fabs(delta_y)){
        delta_x = 1 * delta_x / fabs(delta_y);
        delta_y = 1 * delta_y / fabs(delta_y);
      } else {
        delta_y = 1 * delta_y / fabs(delta_x);
        delta_x = 1 * delta_x / fabs(delta_x);
      }
    }

    if (fabs(delta_theta) > 1)
    {
      delta_theta = 1 * delta_theta/fabs(delta_theta);
    }

    // Write the command
    gazebo_msgs::ModelState state_robot_msg;

    state_robot_msg.model_name = "my_robot";

    state_robot_msg.pose.position.x = posR->transform.translation.x;
    state_robot_msg.pose.position.y = posR->transform.translation.y;
    state_robot_msg.pose.position.z = posR->transform.translation.z;

    state_robot_msg.pose.orientation.x = posR->transform.rotation.x;
    state_robot_msg.pose.orientation.y = posR->transform.rotation.y;     
    state_robot_msg.pose.orientation.z = posR->transform.rotation.z; 
    state_robot_msg.pose.orientation.w = posR->transform.rotation.w; 

    state_robot_msg.twist.linear.x = delta_x;
    state_robot_msg.twist.linear.y = delta_y;
    state_robot_msg.twist.linear.z = 0.0;

    state_robot_msg.twist.angular.x = 0.0;
    state_robot_msg.twist.angular.y = 0.0;
    state_robot_msg.twist.angular.z = delta_theta;

    pub_sim.publish(state_robot_msg);

    //ros::NodeHandle nh;
    if (!ros::Time::waitForValid(ros::WallDuration(5.0))) // NOTE: Important when using simulated clock
    {
      ROS_INFO_STREAM("!!!Timed-out waiting for valid time --------");
    }

    arm_control_client_Ptr ArmClient;
    createArmClient(ArmClient);
    // Generates the goal for the TIAGo's arm
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    waypoints_arm_goal(arm_goal);

    // Sends the command to start the given trajectory 1s from now
    arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);
    ArmClient->sendGoal(arm_goal);
    ROS_INFO_STREAM("send goal");
    // Wait for trajectory execution
    while(!(ArmClient->getState().isDone()) && ros::ok())
    {
      ros::Duration(2.0).sleep(); // sleep for four seconds
    }

    //---------------------------------------

  }

  MultipleSubscribeAndPublishSim()
  {
    pub_sim = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    table1_sub_sim.subscribe(n,"position_table1_sim", 1);
    table2_sub_sim.subscribe(n,"position_table2_sim", 1);
    robot_sub_sim.subscribe(n,"position_robot_sim", 1);
    talos_sub_sim.subscribe(n,"position_talos_sim", 1);
    sync.reset(new Sync(MySyncPolicy(10), table1_sub_sim, table2_sub_sim, robot_sub_sim));      
    sync->registerCallback(boost::bind(&MultipleSubscribeAndPublishSim::callback, this, _1, _2, _3));
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PositionManagerSim");

  MultipleSubscribeAndPublishSim MSPObject;

  ros::spin();

  return 0;
}


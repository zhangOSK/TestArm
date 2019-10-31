#include <ros/ros.h>
#include <pinocchio/fwd.hpp>
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

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <sensor_msgs/JointState.h>

// arm controller action client
#include <string>
#include <boost/shared_ptr.hpp>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>



class MultipleSubscribeAndPublishSim
{
private:
  ros::NodeHandle n; 
  ros::Publisher pub_sim;
  ros::Publisher pub_joint;//test arm
  ros::Publisher pub_whole_joint;

  message_filters::Subscriber<geometry_msgs::TransformStamped> table1_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> table2_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> robot_sub_sim;
  message_filters::Subscriber<geometry_msgs::TransformStamped> talos_sub_sim;
  message_filters::Subscriber<sensor_msgs::JointState> joint_sub_sim;

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TransformStamped,
  geometry_msgs::TransformStamped, geometry_msgs::TransformStamped,
  sensor_msgs::JointState> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

  Eigen::VectorXd qArml, qArml_init;
  Eigen::Vector3d lwPre;

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
  
  void iKwithImpedanceOnLeftArm(Eigen::VectorXd & qArml,
                             Eigen::Vector3d & lwPre,
                             Eigen::Vector3d & aCoMPosition);

  Eigen::Vector3d impHandPos(Eigen::Vector3d & lwPre,
                            Eigen::Vector3d & aCoMPosition,
                            Eigen::Vector3d & lwCur);

  void createArmClient(arm_control_client_Ptr& actionClient);
  void waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal, Eigen::VectorXd & qArml);
                            

  void callback(const geometry_msgs::TransformStamped::ConstPtr& pos1,const geometry_msgs::TransformStamped::ConstPtr& pos2,
  const geometry_msgs::TransformStamped::ConstPtr& posR, const sensor_msgs::JointState::ConstPtr& msg_joint_read)
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

    //----------from whole body config, get left arm joint info-----------
    sensor_msgs::JointState joint_arm;
    std_msgs::Header h = msg_joint_read->header;
    joint_arm.header.stamp = ros::Time::now();
    joint_arm.name.resize(7);
    joint_arm.position.resize(7);

    //at beginning, set half-sitting as initial pose for arm 
    if(qArml(0) == 0 && qArml(1) == 0 &&
       qArml(2) == 0 && qArml(3) == 0   )
    {
      for (int i = 0; i < 7; i++)
      {
        joint_arm.name[i] = msg_joint_read->name[i];
        joint_arm.position[i] = qArml_init(i);
        qArml(i) = qArml_init(i);
      }
    } 
    else //otherwise, read from qArml
    {
      for (int i = 0; i < 7; i++)
      {
        joint_arm.name[i] = msg_joint_read->name[i];
        joint_arm.position[i] = qArml(i);
      }
    }
    
    pub_joint.publish(joint_arm);

    double robot_dist_walk = sqrt(state_robot_msg.pose.position.x * state_robot_msg.pose.position.x +
                           state_robot_msg.pose.position.y * state_robot_msg.pose.position.y);//dist that the box (robot) has walked
    Eigen::Vector3d robot_dist; robot_dist.fill(0);
    robot_dist(0) = robot_dist_walk; //put the dist in the x, cause in experiment, we let robot walk straight.
    iKwithImpedanceOnLeftArm(qArml, lwPre, robot_dist);

    //set qarml to robot
    sensor_msgs::JointState joint_whole;
    joint_whole.header.stamp = h.stamp;
    joint_whole.name.resize(msg_joint_read->name.size());
    joint_whole.position.resize(msg_joint_read->name.size());

    for (int i = 0; i < 7; i++)
    {
      joint_whole.name[i] = msg_joint_read->name[i];
      joint_whole.position[i] = qArml(i);
    }
    for (int i = 7; i < joint_whole.name.size(); i++)
    {
      joint_whole.name[i] = msg_joint_read->name[i];
      joint_whole.position[i] = msg_joint_read->position[i];
    }
    
    pub_whole_joint.publish(joint_whole);

    // ---------------arm control client---------------- 
    if (!ros::Time::waitForValid(ros::WallDuration(2.0))) // NOTE: Important when using simulated clock
    {
      //ROS_INFO_STREAM("!!!Timed-out waiting for valid time --------");
    }

    arm_control_client_Ptr ArmClient;
    createArmClient(ArmClient);
    // Generates the goal for the Talos's arm
    control_msgs::FollowJointTrajectoryGoal arm_goal;
    waypoints_arm_goal(arm_goal, qArml);

    // Sends the command to start the given trajectory 1s from now
    arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1);
    ArmClient->sendGoal(arm_goal);

    while(!(ArmClient->getState().isDone()) && ros::ok())
    {
      ros::Duration(2.0).sleep(); // sleep for four seconds
    }
  }

  MultipleSubscribeAndPublishSim()
  {
    qArml_init.resize(7);
    qArml_init << 0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1;
    qArml.resize(7);
    qArml.fill(0);
    lwPre << 0.0781226, 0.410448, -0.22532;

    pub_whole_joint = n.advertise<sensor_msgs::JointState>("/ang/set_whole_joint_states", 1);//
    pub_joint = n.advertise<sensor_msgs::JointState>("/ang/joint_states", 1);//
    pub_sim = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    
    table1_sub_sim.subscribe(n,"position_table1_sim", 1);
    table2_sub_sim.subscribe(n,"position_table2_sim", 1);
    robot_sub_sim.subscribe(n,"position_robot_sim", 1);
    talos_sub_sim.subscribe(n,"position_talos_sim", 1);
    joint_sub_sim.subscribe(n,"/joint_states", 1);//

    sync.reset(new Sync(MySyncPolicy(10), table1_sub_sim, table2_sub_sim, robot_sub_sim, joint_sub_sim)); //     
    sync->registerCallback(boost::bind(&MultipleSubscribeAndPublishSim::callback, this, _1, _2, _3, _4));//
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PositionManagerSim");

  MultipleSubscribeAndPublishSim MSPObject;

  ros::spin();

  return 0;
}

void MultipleSubscribeAndPublishSim::iKwithImpedanceOnLeftArm(Eigen::VectorXd & qArml,
                              Eigen::Vector3d & lwPre,
                              Eigen::Vector3d & aCoMPosition)
{
  pinocchio::Model modelArm;
  const std::string filename = "/home/ang/Downloads/talos_data/urdf/talos_arm.urdf";
  pinocchio::urdf::buildModel(filename, modelArm);
  pinocchio::Data dataArm(modelArm);
  Eigen::Vector3d xdes;   
  Eigen::VectorXd q;
  q.resize(modelArm.nq,1);
  for (unsigned int k=0; k<q.size(); k++)
  {
    q(k) = qArml(k);
  }
  // To get current lw in shoulder frame
  pinocchio::forwardKinematics(modelArm,dataArm,q);  
  const Eigen::Vector3d & x0   = dataArm.oMi[7].translation();
  Eigen::Vector3d lwCurrentShoulderFrame = x0;

  // Compute target lw pos by impedance control
  xdes = impHandPos(lwPre, aCoMPosition, lwCurrentShoulderFrame); 
  lwPre = lwCurrentShoulderFrame;

  const double eps      = 5e-4;
  const int IT_MAX      = 1000; 
  const double DT       = 5e-3;
  pinocchio::Data::Matrix6x J(6,modelArm.nv); J.setZero(); //7 joint in arm
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;
  Eigen::BDCSVD<pinocchio::Data::Matrix3x> svd(3,modelArm.nv,svdOptions);
  Eigen::Vector3d err;  
  const int    JOINT_ID = 7;
  for (int i=0;;i++)
  {
    pinocchio::forwardKinematics(modelArm,dataArm,q);  
    const Eigen::Vector3d & x   = dataArm.oMi[JOINT_ID].translation(); //oMi indacates world frame
    const Eigen::Matrix3d & R   = dataArm.oMi[JOINT_ID].rotation();
    err = R.transpose() * (x - xdes);
    if(err.norm() < eps)
    {
        for (unsigned int j=0; j<qArml.size(); j++)
        {
          qArml(j) = q(j);
        }
        break;
    }
    if (i >= IT_MAX)
    {
        ROS_INFO_STREAM("!!!IK not reached convergence --------");
        for (unsigned int j=0; j<qArml.size(); j++)
        {
          qArml(j) = q(j);
        }
        break;
    }
    pinocchio::jointJacobian(modelArm,dataArm,q,JOINT_ID,J);;
    const Eigen::VectorXd v     = - svd.compute(J.topRows<3>()).solve(err);
    ROS_INFO_STREAM("v0="<<v(0)<<", v1="<<v(1)<<", v2="<<v(2)<<", v3="<<v(3)<<", v4="<<v(4)<<", v5="<<v(5)<<", v6="<<v(6));
    if(v(0)>100 || v(1)>100 || v(2)>100 || v(3)>100 || v(4)>100 || v(5)>100 || v(6)>100)
    {ROS_INFO_STREAM("-------!!!Big veloctiy!!!--------");}
    q = pinocchio::integrate(modelArm,q,v*DT);
  }
}

Eigen::Vector3d MultipleSubscribeAndPublishSim::impHandPos(Eigen::Vector3d & lwPre,
                            Eigen::Vector3d & aCoMPosition,
                            Eigen::Vector3d & lwCur)
{
  Eigen::Vector3d lwDes, flw, fd, fDS, flw_shoulder;
  double vel_ = 0.1;
  lwDes.fill(0);
  flw.fill(0); //Real force collectted from left wrist sensor
  fd.fill(0);  //Desired force
  fDS.fill(0);
  flw_shoulder.fill(0);
  double waistInitX = 0.0;
  double distWaistX = aCoMPosition(0) - waistInitX;
  double flwX = 14 * distWaistX + 5; //Imitate real hand force.
  flw(0) = flwX;
  flw_shoulder = flw;

  //robot pulls only at DS;;; Currently, just pull if hand is far behind body  -0.03, 0.1
  if(lwPre(0) < -18 || (lwCur(0) - lwPre(0)) > 0)
  {
    fDS(0) = vel_ * 27000;
  }
  if(lwPre(0) > 18 && (lwCur(0) - lwPre(0)) >= 0)
  {
    fDS(0) = 0.0;
  }
  
  //impedance param
  double massHose = 7.04, part = 0.32;
  double dt = 0.005;
  double mu = 0.5;
  double gx = -9.8;
  fd(0) = mu * (0.4 * massHose) * gx;
  fd(1) = 0.0;
  fd(2) = (part * massHose * gx) -11.0; //Sensor offset = -11.0 
  double m_ = 1.0, c_ = 5.0;

  Eigen::Vector3d posImp;
  posImp.fill(0);
  //posImp = (((dt * dt) / m_) * (flw_shoulder - (fd - fDS) - ((c_ / dt) * (lwCurPos - lwLoPre)))) 
  //         + 2 * lwCurPos - lwLoPre;
  posImp = (((dt * dt) / m_) * (-1.1*flw_shoulder + 0.06*fDS - ((c_ / dt) * (lwCur - lwPre)))) 
           + 2 * lwCur - lwPre;
  posImp(1) = lwPre(1);
  posImp(2) = lwPre(2);

  lwDes = posImp;

  if (lwDes(0) > 0.2)
  {
    lwDes(0) = 0.2 ; //the X limit distance between lw and waist is 20cm
  }
  if (lwDes(0)  < -0.2)
  {
    lwDes(0) = -0.2 ; 
  }
  return lwDes;
}


void MultipleSubscribeAndPublishSim::createArmClient(arm_control_client_Ptr& actionClient)
{
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

// Generates a simple trajectory with two waypoints to move Talos' arm 
void MultipleSubscribeAndPublishSim::waypoints_arm_goal(control_msgs::FollowJointTrajectoryGoal& goal, Eigen::VectorXd & qArml)
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
  goal.trajectory.points.resize(2);

  //0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1
  // First trajectory point
  int index = 0;
  goal.trajectory.points[index].positions.resize(7);

  goal.trajectory.points[index].positions[0] = qArml(0);
  goal.trajectory.points[index].positions[1] = qArml(1);
  goal.trajectory.points[index].positions[2] = qArml(2);
  goal.trajectory.points[index].positions[3] = qArml(3);
  goal.trajectory.points[index].positions[4] = qArml(4);
  goal.trajectory.points[index].positions[5] = qArml(5);
  goal.trajectory.points[index].positions[6] = qArml(6);

  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 1.0; //1.0
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(1);
  
  index += 1;
  goal.trajectory.points[index].positions.resize(7);
  /*goal.trajectory.points[index].positions[0] = 0.25;
  goal.trajectory.points[index].positions[1] = 0.17;
  goal.trajectory.points[index].positions[2] = 0.0;
  goal.trajectory.points[index].positions[3] = -0.5;
  goal.trajectory.points[index].positions[4] = 0.0;
  goal.trajectory.points[index].positions[5] = -0.0;
  goal.trajectory.points[index].positions[6] = 0.1;*/
  
  goal.trajectory.points[index].positions[0] = qArml(0);
  goal.trajectory.points[index].positions[1] = qArml(1);
  goal.trajectory.points[index].positions[2] = qArml(2);
  goal.trajectory.points[index].positions[3] = qArml(3);
  goal.trajectory.points[index].positions[4] = qArml(4);
  goal.trajectory.points[index].positions[5] = qArml(5);
  goal.trajectory.points[index].positions[6] = qArml(6);
  // Velocities
  goal.trajectory.points[index].velocities.resize(7);
  for (int j = 0; j < 7; ++j)
  {
    goal.trajectory.points[index].velocities[j] = 0.0;
  }
  // To be reached 2 second after starting along the trajectory
  goal.trajectory.points[index].time_from_start = ros::Duration(2);
  
  
}
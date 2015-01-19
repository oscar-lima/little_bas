/*
* bas arm driver tester
*
* Author: Oscar Lima (olima_84@yahoo.com)
* 
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <bas_manipulation_msgs/RawArmGoal.h>
#include <bas_arm_driver_tester/bas_arm_driver_testerConfig.h>

ros::Publisher arm_pub;

void dynamicReconfigureCallback (bas_arm_driver_tester::bas_arm_driver_testerConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure request received...");
	
	bas_manipulation_msgs::RawArmGoal arm1;
	arm1.header.frame_id = "arm1";
	arm1.number_of_joints = 6;
	
	arm1.motor_ids.push_back(0);
	arm1.motor_ids.push_back(1);
	arm1.motor_ids.push_back(2);
	arm1.motor_ids.push_back(3);
	arm1.motor_ids.push_back(4);
	arm1.motor_ids.push_back(5);
	
	arm1.goal_pos.push_back(config.joint0_value);
	arm1.goal_pos.push_back(config.joint1_value);
	arm1.goal_pos.push_back(config.joint2_value);
	arm1.goal_pos.push_back(config.joint3_value);
	arm1.goal_pos.push_back(config.joint4_value);
	arm1.goal_pos.push_back(config.joint5_value);
	
	arm_pub.publish(arm1);
	
}
	
int main (int argc, char **argv)
{
	ros::init (argc, argv, "bas_arm_driver_tester");
	ROS_INFO ("bas_arm_driver_tester node initialized ...");
	
	ros::NodeHandle nh ("~");
	arm_pub = nh.advertise<bas_manipulation_msgs::RawArmGoal>("/bas_arm_driver/raw_arm_goal", 1);
	
	dynamic_reconfigure::Server<bas_arm_driver_tester::bas_arm_driver_testerConfig> server;
	dynamic_reconfigure::Server<bas_arm_driver_tester::bas_arm_driver_testerConfig>::CallbackType f;
	
	f = boost::bind (&dynamicReconfigureCallback, _1, _2);
	server.setCallback (f);
	
	ros::spin ();
}
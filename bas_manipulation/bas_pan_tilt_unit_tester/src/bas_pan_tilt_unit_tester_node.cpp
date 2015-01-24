/*
* bas pan tilt unit tester: moves the pan tilt unit camera
* with asus on 2 axis
*
* Author: Oscar Lima (olima_84@yahoo.com)
* 
*/

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <bas_manipulation_msgs/RawArmGoal.h>
#include <bas_pan_tilt_unit_tester/bas_pan_tilt_unit_testerConfig.h>

ros::Publisher pan_tilt_publisher;

void dynamicReconfigureCallback (bas_pan_tilt_unit_tester::bas_pan_tilt_unit_testerConfig &config, uint32_t level)
{
	ROS_INFO("Reconfigure request received...");
	
	bas_manipulation_msgs::RawArmGoal pan_tilt_unit;
	pan_tilt_unit.header.frame_id = "pan_tilt_unit";
	pan_tilt_unit.number_of_joints = 2;
	
	pan_tilt_unit.motor_ids.push_back(6);
	pan_tilt_unit.motor_ids.push_back(7);
	
	pan_tilt_unit.goal_pos.push_back(config.joint0_value);
	pan_tilt_unit.goal_pos.push_back(config.joint1_value);
	
	pan_tilt_publisher.publish(pan_tilt_unit);
	
}
	
int main (int argc, char **argv)
{
	ros::init (argc, argv, "bas_pan_tilt_unit_tester");
	ROS_INFO ("bas_pan_tilt_unit_tester node initialized ...");
	
	ros::NodeHandle nh ("~");
	
	// last two joints of the arm are considered to be the joints 0 and 1 of the pan tilt unit
	pan_tilt_publisher = nh.advertise<bas_manipulation_msgs::RawArmGoal>("/bas_arm_driver/raw_arm_goal", 1);
	
	dynamic_reconfigure::Server<bas_pan_tilt_unit_tester::bas_pan_tilt_unit_testerConfig> server;
	dynamic_reconfigure::Server<bas_pan_tilt_unit_tester::bas_pan_tilt_unit_testerConfig>::CallbackType f;
	
	f = boost::bind (&dynamicReconfigureCallback, _1, _2);
	server.setCallback (f);
	
	ros::spin ();
}
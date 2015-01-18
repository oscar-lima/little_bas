#include "ros/ros.h"
#include "std_msgs/String.h"
#include "bas_manipulation_msgs/RawArmGoal.h"
#include <l6ac-kt_arm_driver.h>

BasArm arm1;

void RawarmGoalCallback(const bas_manipulation_msgs::RawArmGoal::ConstPtr& msg)
{
	bas_manipulation_msgs::RawArmGoal arm_goal;
	arm_goal = *msg;
	
	std::cout << "message received" << std::endl;
	
	arm1.move_many_joints(&arm_goal.motor_ids[0], &arm_goal.goal_pos[0],
						  arm_goal.number_of_joints);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bas_arm_driver");
	ROS_INFO("Arm driver node initialized...");
	ros::NodeHandle nh("~");
	ros::Subscriber sub = nh.subscribe("raw_arm_goal", 1, RawarmGoalCallback);
	
	std::string arm_port_name;
	
	//getting arm id from parameter server
	nh.param<std::string>("arm_port_name", arm_port_name, "/dev/ttyUSB0");
	
	//setup arm
	arm1.setup_arm("/dev/ttyUSB0", true);
	
	//initialize bas arm
	arm1.init_arm();
	
	ros::spin();

	return 0;
}
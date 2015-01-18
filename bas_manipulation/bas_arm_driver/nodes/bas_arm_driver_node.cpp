#include "ros/ros.h"
#include "std_msgs/String.h"
#include <l6ac-kt_arm_driver.h>

BasArm arm1("/dev/ttyUSB0", true);

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("testing bas arm: [%s]", msg->data.c_str());
	int angle = atoi(msg->data.c_str());
	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bas_arm_driver");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("test_bas_arm", 1, chatterCallback);
	
	//initialize bas arm
	arm1.init_arm();
	
	int joint = 0;
	int angle = 80;
	
	//move arm to home position
	arm1.move_one_joint(&joint, &angle);
	
	ros::spin();

	return 0;
}
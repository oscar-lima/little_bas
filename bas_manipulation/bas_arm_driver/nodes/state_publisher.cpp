#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "bas_arm_state_publisher");
	ros::NodeHandle nh;
	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("bas_arm_joint_states", 1);
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(10);

	// robot state
	double joint0 = 0.0, joint1 = 0.0, joint2 = -0.74;

	// message declarations
	sensor_msgs::JointState joint_state;
	geometry_msgs::TransformStamped arm_trans;
	arm_trans.header.frame_id = "arm_base";
    arm_trans.child_frame_id = "arm_link1";
	
	while (ros::ok())
	{
		//update joint_state
		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(3);
		joint_state.position.resize(3);
		joint_state.name[0] ="arm_base";
		joint_state.position[0] = joint0;
		joint_state.name[1] ="arm_link1";
		joint_state.position[1] = joint1;
		joint_state.name[2] ="arm_link2";
		joint_state.position[2] = joint2;
		
		arm_trans.header.stamp = ros::Time::now();
		//arm_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0.75);
		arm_trans.transform.rotation.x = 0.0;
		arm_trans.transform.rotation.y = 0.0;
		arm_trans.transform.rotation.z = 0.0;
		arm_trans.transform.rotation.w = 1.0;
		
		//send the joint state and transform
		joint_pub.publish(joint_state);
		broadcaster.sendTransform(arm_trans);

		// This will adjust as needed per iteration
		loop_rate.sleep();
	}

	return 0;
}
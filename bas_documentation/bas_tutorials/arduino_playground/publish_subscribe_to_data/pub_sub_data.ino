/*
 * This program is a simple publisher and subscriber to the
 * ROS network.
 * 
 * It receives a float64 std message, adds 5 and returns the
 * result.
 * 
 * subscribes to /test topic
 * publishes on /number_back topic
 * 
 * Author: Oscar Lima (olima_84@yahoo.com)
 * 
 * with information from:
 * https://gist.github.com/Bad-Robot/2158748
 * 
 * 
 */

#include <ros.h>
#include <std_msgs/Float64.h>
 
ros::NodeHandle nh;
 
std_msgs::Float64 float64_msg;
ros::Publisher float64_number("number_back", &float64_msg);
 
void messageCb( const std_msgs::Float64& incoming_msg )
{
	double hola = 0;
	hola = incoming_msg.data;
	hola = hola + 5;
	float64_msg.data = hola;
	float64_number.publish( &float64_msg );
}
 
ros::Subscriber<std_msgs::Float64> sub("test", &messageCb );
 
void setup()
{
	nh.initNode();
	nh.advertise(float64_number);
	nh.subscribe(sub);
}
 
void loop()
{
	nh.spinOnce();
	delay(200);
} 
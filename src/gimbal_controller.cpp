#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	std::cout << "Hello, ROS World!" << std::endl;

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::spin();

	//read marker array topic from whycon
	//determine if any of the markers are 2-bit whycon marker with id 1 or 2
	//	id checking may be necessary to determine yaw with rotationally
	//	symmetric ids 1 and 2
	//determine if any of the markers are apriltags with id 489 (420 + 69 nice)
	//if time since marker detection exceeds some time t_epsilon
	//	set gimbal rotation back to (0, 0)
	//	OR
	//	search the field of view for more markers
	//else
	//	determine optimal viewing angle
	//		if only a single marker is visible
	//			put it to the center of the view
	//		if both markers are visible
	//			put the center of the view somewhere on a line between the markers,
	//			closer to the one with the larger area
	//	Maintain 2-d PID controller to attempt to set the target point in the middle

	return 0;
}

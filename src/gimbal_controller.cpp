#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>

#include <cmath>
#include <ctime>

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

std_msgs::Float64 setpoint_x;
std_msgs::Float64 setpoint_y;

ros::Publisher setpoint_publisher_x;
ros::Publisher setpoint_publisher_y;

void visual_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	int num_markers = msg->markers.size();
	double x, y, z;
	double rotation_x, rotation_y, rotation_z, rotation_w;
	ROS_INFO("Detected %d marker(s).", num_markers);

	for(int i = 0; i < num_markers; i ++)
	{
		int id = msg->markers[i].id;

		x = msg->markers[i].pose.position.x;
		y = msg->markers[i].pose.position.y;
		z = msg->markers[i].pose.position.z;

		rotation_x = msg->markers[i].pose.orientation.x;
		rotation_y = msg->markers[i].pose.orientation.y;
		rotation_z = msg->markers[i].pose.orientation.z;
		rotation_w = msg->markers[i].pose.orientation.w;

		ROS_INFO("\n\tID: %d", id);
		ROS_INFO("\n\tPose: (%0.3f, %0.3f, %0.3f)\n\tRotation:(%0.3f, %0.3f, %0.3f, %0.3f)",
			x,
			y,
			z,
			rotation_x,
			rotation_y,
			rotation_z,
			rotation_w);
	}
	
	setpoint_x.data =  - atan(x / z);
	setpoint_y.data =  atan(y / z);

	setpoint_publisher_x.publish(setpoint_x);
	setpoint_publisher_y.publish(setpoint_y);

	ROS_INFO("setpoint = (%f, %f)", setpoint_x.data, setpoint_y.data);
}

int main(int argc, char **argv)
{
	ROS_INFO("Started gimbal_controller node.");

	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	
	// Subscriber to get position(s) of landing pad marker(s)
	ros::Subscriber visual_subscriber = node_handle.subscribe("/whycon_ros/visual", 1000, visual_callback);
	
	// Publisher to set the states of the current PID control parameters
	setpoint_publisher_x = node_handle.advertise<std_msgs::Float64>("/iris/camera/x/setpoint", 1000);
	setpoint_publisher_y = node_handle.advertise<std_msgs::Float64>("/iris/camera/y/setpoint", 1000);

	ros::spin();

	return 0;
}
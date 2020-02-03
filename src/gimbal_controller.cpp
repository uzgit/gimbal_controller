#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/LinkState.h>

ros::Publisher publisher;

void callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	int num_markers = msg->markers.size();
	double x, y, z;
	double rotation_x, rotation_y, rotation_z, rotation_w;
	ROS_INFO("Detected %d markers.", num_markers);

	for(int i = 0; i < 1; i ++)
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

	gazebo_msgs::LinkState link_state_msg;
	link_state_msg.link_name = "camera_link";
	link_state_msg.pose.orientation.x = rotation_x;
	link_state_msg.pose.orientation.y = rotation_y;
	link_state_msg.pose.orientation.z = rotation_z;
	link_state_msg.pose.orientation.w = rotation_w;
//	link_state_msg.reference_frame = "parent";
	publisher.publish(link_state_msg);
	
}

int main(int argc, char **argv)
{
	ROS_INFO("Started gimbal_controller node.");

	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	ros::Subscriber subscriber = node_handle.subscribe("/whycon_ros/visual", 1000, callback);
	//ros::Publisher
	publisher  = node_handle.advertise<gazebo_msgs::LinkState>("test", 1000);
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

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>

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

int image_height = 480;
int image_width  = 640;

bool new_data_flag;
gazebo_msgs::LinkState link_state_msg;
ros::Publisher link_state_publisher;

void visual_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	int num_markers = msg->markers.size();
	double x, y, z;
	double rotation_x, rotation_y, rotation_z, rotation_w;
//	ROS_INFO("Detected %d marker(s).", num_markers);

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
/*
		ROS_INFO("\n\tID: %d", id);
		ROS_INFO("\n\tPose: (%0.3f, %0.3f, %0.3f)\n\tRotation:(%0.3f, %0.3f, %0.3f, %0.3f)",
			x,
			y,
			z,
			rotation_x,
			rotation_y,
			rotation_z,
			rotation_w);
*/
	}
/*
	link_state_msg.pose.position.x = x;
	link_state_msg.pose.position.y = y;
	link_state_msg.pose.position.z = z;
*/


	link_state_msg.pose.orientation.x = (x - image_width/2) * 0.1;
	link_state_msg.pose.orientation.y = (y - image_width/2) * 0.1;

	ROS_INFO("x_rotation: %s", link_state_msg.pose.orientation.x);
	ROS_INFO("y_rotation: %s", link_state_msg.pose.orientation.y);

//	link_state_msg.pose.orientation.z = 0;
//	link_state_msg.pose.orientation.w = 0;

//	new_data_flag = true;
}

void link_states_callback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
	int rotation_x, rotation_y;

	int i = 0;
	while( i < msg->name.size() && msg->name[i] != link_state_msg.link_name)
	{
		i ++;
	}
	if( i < msg->name.size() )
	{
		rotation_x = link_state_msg.pose.orientation.x;
		rotation_y = link_state_msg.pose.orientation.y;

//		ROS_INFO("setting link state...");
		link_state_msg.pose = msg->pose[i];
		link_state_msg.twist = msg->twist[i];
		if( new_data_flag )
		{
			// Reset x and y to the previously calculated values
//			link_state_msg.pose.orientation.y = rotation_x;
//			link_state_msg.pose.orientation.z = rotation_y;

			new_data_flag = false;
		}
		link_state_publisher.publish(link_state_msg);
	}
}

int main(int argc, char **argv)
{
	ROS_INFO("Started gimbal_controller node.");

	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	
	// Subscriber to get position(s) of landing pad marker(s)
	new_data_flag = false;
	ros::Subscriber visual_subscriber = node_handle.subscribe("/whycon_ros/visual", 1000, visual_callback);
	// Subscriber to get current link states from Gazebo
	ros::Subscriber link_state_subscriber = node_handle.subscribe("/gazebo/link_states", 1000, link_states_callback);

	// Publisher to set the states of the current PID control parameters
//	ros::Publisher = node_handle.advertise<

	// Publisher to set the target link state (rotation) of the camera
	link_state_publisher = node_handle.advertise<gazebo_msgs::LinkState>("/gazebo/set_link_state", 1000);

	// initialize link_state_msg
	link_state_msg.link_name = "iris_demo::iris_demo::camera_link";
//	link_state_msg.link_name = "iris_demo::iris_demo::gimbal_small_2d::tilt_link";
	link_state_msg.reference_frame = "world"; //"iris_demo::iris_demo::gimbal_small_2d::tilt_link";

/*
	ros::Rate loop_rate(30);
	while( ros::ok() )
	{
		ROS_INFO("ran loop");

		ros::spinOnce();
		loop_rate.sleep();
	}
*/
	ros::spin();

	return 0;
}

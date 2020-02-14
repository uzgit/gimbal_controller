#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <sensor_msgs/Imu.h>

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

int landing_pad_id[2] = {1, 2};

double idle_setpoint_x = 0.0;
double idle_setpoint_y = 0.4;

std_msgs::Float64 setpoint_x;
std_msgs::Float64 setpoint_y;
std_msgs::Bool idle_state_msg;

ros::Publisher setpoint_publisher_x;
ros::Publisher setpoint_publisher_y;
ros::Publisher idle_state_publisher;

ros::Time last_detection_time(0);

void visual_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	bool detection = false;

	int num_markers = msg->markers.size();
	double x, y, z;
	double rotation_x, rotation_y, rotation_z, rotation_w;
//	ROS_INFO("Detected %d marker(s).", num_markers);

	int i = 0;
	while( i < num_markers && ! detection )
	{
		int id = msg->markers[i].id;

//		ROS_INFO("id: %d", id);

		x = msg->markers[i].pose.position.x;
		y = msg->markers[i].pose.position.y;
		z = msg->markers[i].pose.position.z;

		rotation_x = msg->markers[i].pose.orientation.x;
		rotation_y = msg->markers[i].pose.orientation.y;
		rotation_z = msg->markers[i].pose.orientation.z;
		rotation_w = msg->markers[i].pose.orientation.w;

//		ROS_INFO("\n\tID: %d", id);
//		ROS_INFO("\n\tPose: (%0.3f, %0.3f, %0.3f)\n\tRotation:(%0.3f, %0.3f, %0.3f, %0.3f)",
//			x,
//			y,
//			z,
//			rotation_x,
//			rotation_y,
//			rotation_z,
//			rotation_w);
//
		int ii = 0;
		while( ii < 2 && landing_pad_id[ii] != id )
		{
			ii ++;
		}
		if( ii < 2 )
		{
			detection = true;
		}

		i ++;
	}

	if( detection )
	{
		// for setting absolute position
		setpoint_x.data = - atan( 4 * x / z );
		setpoint_y.data = atan( 4 * y / z );

		// for setting velocity
	//	setpoint_x.data = -x;
	//	setpoint_y.data = y;

		setpoint_publisher_x.publish(setpoint_x);
		setpoint_publisher_y.publish(setpoint_y);
		idle_state_msg.data = false;

		last_detection_time = ros::Time::now();

	//	ROS_INFO("setpoint = (%f, %f)", setpoint_x.data, setpoint_y.data);
	}
}

int main(int argc, char **argv)
{
	ROS_INFO("Started gimbal_controller!");

	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	
	// Subscriber to get position(s) of landing pad marker(s)
	ros::Subscriber visual_subscriber     = node_handle.subscribe("/whycon_ros/visual", 1000, visual_callback);
	
	// Publisher to set the states of the current PID control parameters
	setpoint_publisher_x = node_handle.advertise<std_msgs::Float64>("/iris/camera/x/setpoint", 1000);
	setpoint_publisher_y = node_handle.advertise<std_msgs::Float64>("/iris/camera/y/setpoint", 1000);
	idle_state_publisher = node_handle.advertise<std_msgs::Bool>("/iris/camera/idle_state",1000);

	idle_state_msg.data = false;
	// initialize gimbal position to forward level
	setpoint_x.data = 0;
	setpoint_y.data = 0;
	setpoint_publisher_x.publish(setpoint_x);
	setpoint_publisher_y.publish(setpoint_y);

	ros::Duration detection_fail_timeout(2.0);
	ros::Rate loop_rate(70);

	while( ros::ok() )
	{
		ros::spinOnce();

		if( ros::Time::now() - last_detection_time > detection_fail_timeout && ! idle_state_msg.data)
		{
			// set the gimbal to the idle attitude
			setpoint_x.data = idle_setpoint_x;
			setpoint_y.data = idle_setpoint_y;
			idle_state_msg.data = true;
			
			// publish the message
			setpoint_publisher_x.publish(idle_setpoint_x);
			setpoint_publisher_y.publish(idle_setpoint_y);

			idle_state_publisher.publish(idle_state_msg);
		}

		loop_rate.sleep();
	}

	return 0;
}

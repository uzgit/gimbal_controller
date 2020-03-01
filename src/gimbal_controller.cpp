#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <ctime>

int landing_pad_id[2] = {1, 2};

double idle_setpoint_x = 0.0;
double idle_setpoint_y = 0.4;

double gimbal_x_position = std::nan("1");
double gimbal_y_position = std::nan("1");

std_msgs::Float64 setpoint_x;
std_msgs::Float64 setpoint_y;
std_msgs::Bool idle_state_msg;

ros::Publisher setpoint_publisher_x;
ros::Publisher setpoint_publisher_y;
ros::Publisher idle_state_publisher;

ros::Time last_detection_time(0);

tf2_ros::Buffer transform_buffer;

void gimbal_x_position_callback(const std_msgs::Float64::ConstPtr msg)
{
	gimbal_x_position = msg->data;
}

void gimbal_y_position_callback(const std_msgs::Float64::ConstPtr msg)
{
	gimbal_y_position = msg->data;
}

void visual_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	

	bool detection = false;

	int num_markers = msg->markers.size();
	double x, y, z, normal_x, normal_y, normal_z;
	double rotation_x, rotation_y, rotation_z, rotation_w;

	int i = 0;
	while( i < num_markers && ! detection )
	{
		int id = msg->markers[i].id;

		x = msg->markers[i].pose.position.x;
		y = msg->markers[i].pose.position.y;
		z = msg->markers[i].pose.position.z;

		rotation_x = msg->markers[i].pose.orientation.x;
		rotation_y = msg->markers[i].pose.orientation.y;
		rotation_z = msg->markers[i].pose.orientation.z;
		rotation_w = msg->markers[i].pose.orientation.w;

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
/*
		// this works but is very noisy
		static tf2_ros::TransformBroadcaster transform_broadcaster;

		// for getting distance
		geometry_msgs::TransformStamped detection_normal_transform_stamped;
		detection_normal_transform_stamped.header.stamp = ros::Time::now();
		detection_normal_transform_stamped.header.frame_id = "camera_frame";
		detection_normal_transform_stamped.child_frame_id  = "detection_normal_frame";
		detection_normal_transform_stamped.transform.translation.x = 0;
		detection_normal_transform_stamped.transform.translation.y = 0;
		detection_normal_transform_stamped.transform.translation.z = 0;
		detection_normal_transform_stamped.transform.rotation.w = rotation_w;
		detection_normal_transform_stamped.transform.rotation.x = rotation_x;
		detection_normal_transform_stamped.transform.rotation.y = rotation_y;
		detection_normal_transform_stamped.transform.rotation.z = rotation_z;
		
		transform_broadcaster.sendTransform(detection_normal_transform_stamped);

		geometry_msgs::PoseStamped detection_pose;
		detection_pose.header.stamp = ros::Time::now();
		detection_pose.header.frame_id = "camera_frame";
		detection_pose.pose.position.x = x;
		detection_pose.pose.position.y = y;
		detection_pose.pose.position.z = z;
		detection_pose.pose.orientation.w = rotation_w;
		detection_pose.pose.orientation.x = rotation_x;
		detection_pose.pose.orientation.y = rotation_y;
		detection_pose.pose.orientation.z = rotation_z;

		geometry_msgs::PoseStamped detection_normal_pose;

		try
		{
			detection_normal_pose = transform_buffer.transform(detection_pose, "detection_normal_frame", ros::Duration(0.1));
			normal_z = detection_normal_pose.pose.position.z;
			normal_x = copysign(detection_normal_pose.pose.position.x, x);
			normal_y = copysign(detection_normal_pose.pose.position.y, y);
			ROS_INFO_STREAM(detection_pose);
			ROS_INFO_STREAM(detection_normal_pose);
//			ROS_INFO("distance: %0.2f, %0.2f, ", );
		}
		catch( tf2::TransformException &exception)
		{
			ROS_WARN("%s", exception.what());
		}
*/
		// for setting absolute position
//		setpoint_x.data += -0.5 * atan( x / z );
//		setpoint_y.data += 0.5 * atan( y / z );
//		setpoint_x.data += -1.5 * atan( x / z );
//		setpoint_y.data +=  1.5 * atan( y / z );
//		setpoint_x.data =  x * atan( x / z );
//		setpoint_y.data =  -y * atan( y / z );

		// should work in theory but has issues with large angles
//		setpoint_x.data =  -atan( normal_x / normal_z );
//		setpoint_y.data =   atan( normal_y / normal_z );

//		setpoint_x.data += -0.25 * normal_x / normal_z;
//		setpoint_y.data +=  0.25 * normal_y / normal_z;

		// this sort of thing seems to work the best using PID systems on the x and y positions
		setpoint_x.data += -0.1 * x / z;
		setpoint_y.data +=  0.1 * y / z;

//		setpoint_x.data = -x / normal_distance;
//		setpoint_y.data =  y / z;


//		setpoint_x.data += -x / z;
//		setpoint_y.data += y / z;

		// for setting velocity
//		setpoint_x.data = 10 * -x / z;
//		setpoint_y.data = 10 * y / z;
//		setpoint_x.data = -25 * x / z;
//		setpoint_y.data = 25 * y / z;

		setpoint_publisher_x.publish(setpoint_x);
		setpoint_publisher_y.publish(setpoint_y);
		idle_state_msg.data = false;

		last_detection_time = ros::Time::now();
	}
/*	
	else
	{
		setpoint_x.data = 0;
		setpoint_y.data = 0;
	}
*/
}

int main(int argc, char **argv)
{
	ROS_INFO("Started gimbal_controller!");

	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	
	// subscriber to get position(s) of landing pad marker(s)
	ros::Subscriber visual_subscriber = node_handle.subscribe("/whycon_ros/visual", 1000, visual_callback);
	
	// publisher to set the states of the current PID control parameters
	setpoint_publisher_x = node_handle.advertise<std_msgs::Float64>("/gimbal/x/setpoint", 1000);
	setpoint_publisher_y = node_handle.advertise<std_msgs::Float64>("/gimbal/y/setpoint", 1000);
	idle_state_publisher = node_handle.advertise<std_msgs::Bool>("/gimbal/idle_state",1000);
	idle_state_msg.data = false;

	ros::Subscriber gimbal_x_position_subscriber = node_handle.subscribe("/gimbal/x/position", 1000, gimbal_x_position_callback);
	ros::Subscriber gimbal_y_position_subscriber = node_handle.subscribe("/gimbal/y/position", 1000, gimbal_y_position_callback);

	// base_link to camera_frame transform publisher
	static tf2_ros::TransformBroadcaster transform_broadcaster;

	tf2_ros::TransformListener transform_listener(transform_buffer);

	// initialize gimbal position to forward level
	setpoint_x.data = idle_setpoint_x;
	setpoint_y.data = idle_setpoint_y;
	setpoint_publisher_x.publish(setpoint_x);
	setpoint_publisher_y.publish(setpoint_y);

	// timing
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

		// send transform
		geometry_msgs::TransformStamped camera_transform_stamped;
		camera_transform_stamped.header.stamp = ros::Time::now();
		camera_transform_stamped.header.frame_id = "base_link";
		camera_transform_stamped.child_frame_id  = "camera_frame";
		camera_transform_stamped.transform.translation.x = 0;
		camera_transform_stamped.transform.translation.y = 0;
		camera_transform_stamped.transform.translation.z = 0;
		tf2::Quaternion camera_rotation;
		camera_rotation.setRPY(0, gimbal_y_position, gimbal_x_position);

		camera_transform_stamped.transform.rotation.w = camera_rotation.w();
		camera_transform_stamped.transform.rotation.x = camera_rotation.x();
		camera_transform_stamped.transform.rotation.y = camera_rotation.y();
		camera_transform_stamped.transform.rotation.z = camera_rotation.z();
		
		transform_broadcaster.sendTransform(camera_transform_stamped);

		loop_rate.sleep();
	}

	return 0;
}

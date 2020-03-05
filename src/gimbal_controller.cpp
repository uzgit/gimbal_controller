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

#include <apriltag_ros/AprilTagDetectionArray.h>

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
ros::Publisher landing_pad_camera_pose_publisher;

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

void apriltag_visual_callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& _msg)
{
	apriltag_ros::AprilTagDetection detection;

	int num_detections = _msg->detections.size();

	for(int i = 0; i < num_detections; i ++)
	{
		if( _msg->detections[i].id[0] == 0 )
		{
			detection = _msg->detections[i];

			geometry_msgs::PoseWithCovarianceStamped buffer = detection.pose;
			geometry_msgs::PoseStamped apriltag_camera_pose;
			apriltag_camera_pose.pose = buffer.pose.pose;
			apriltag_camera_pose.header.stamp = ros::Time::now();
			apriltag_camera_pose.header.frame_id = "camera_frame";
			landing_pad_camera_pose_publisher.publish(apriltag_camera_pose);

			setpoint_x.data += -0.1 * apriltag_camera_pose.pose.position.x / apriltag_camera_pose.pose.position.z;
			setpoint_y.data +=  0.1 * apriltag_camera_pose.pose.position.y / apriltag_camera_pose.pose.position.z;

			setpoint_publisher_x.publish(setpoint_x);
			setpoint_publisher_y.publish(setpoint_y);
			idle_state_msg.data = false;

			last_detection_time = ros::Time::now();
		}
	}
}

void whycon_visual_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	// capture the pose
	geometry_msgs::PoseStamped landing_pad_camera_pose;	
	landing_pad_camera_pose.header.stamp = ros::Time::now();
	landing_pad_camera_pose.header.frame_id = "camera_frame";
	landing_pad_camera_pose.pose = msg->markers[0].pose;
/*
	landing_pad_camera_pose.pose.position = msg->markers[0].pose.position;
	landing_pad_camera_pose.pose.orientation.w = 1;
	landing_pad_camera_pose.pose.orientation.x = 0;
	landing_pad_camera_pose.pose.orientation.y = 0;
	landing_pad_camera_pose.pose.orientation.z = 0;
*/
	landing_pad_camera_pose_publisher.publish(landing_pad_camera_pose);

	// this sort of thing seems to work the best using PID systems on the x and y positions
	setpoint_x.data += -0.1 * landing_pad_camera_pose.pose.position.x / landing_pad_camera_pose.pose.position.z;
	setpoint_y.data +=  0.1 * landing_pad_camera_pose.pose.position.y / landing_pad_camera_pose.pose.position.z;

	setpoint_publisher_x.publish(setpoint_x);
	setpoint_publisher_y.publish(setpoint_y);
	idle_state_msg.data = false;

	last_detection_time = ros::Time::now();
}

int main(int argc, char **argv)
{
	ROS_INFO("Started gimbal_controller!");

	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	
	// subscriber to get position(s) of landing pad marker(s)
	ros::Subscriber whycon_visual_subscriber	= node_handle.subscribe("/whycon_ros/visual",	1000, whycon_visual_callback);
	ros::Subscriber apriltag_visual_subscriber	= node_handle.subscribe("/tag_detections",	1000, apriltag_visual_callback);
	
	// publisher to set the states of the current PID control parameters
	setpoint_publisher_x = node_handle.advertise<std_msgs::Float64>("/gimbal/x/setpoint", 1000);
	setpoint_publisher_y = node_handle.advertise<std_msgs::Float64>("/gimbal/y/setpoint", 1000);
	idle_state_publisher = node_handle.advertise<std_msgs::Bool>("/gimbal/idle_state",1000);
	idle_state_msg.data = false;

	// publisher to share the pose of the landing pad within the camera frame
	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1000);

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
		camera_transform_stamped.header.frame_id = "body_END";
		camera_transform_stamped.child_frame_id  = "camera_frame";
		camera_transform_stamped.transform.translation.x = 0;
		camera_transform_stamped.transform.translation.y = 0;
		camera_transform_stamped.transform.translation.z = 0;
		tf2::Quaternion camera_rotation;
		camera_rotation.setRPY(0, gimbal_y_position, gimbal_x_position);

		tf2::Quaternion camera_rotation_inverse = camera_rotation.inverse();

		camera_transform_stamped.transform.rotation.w = camera_rotation_inverse.w();
		camera_transform_stamped.transform.rotation.x = camera_rotation_inverse.x();
		camera_transform_stamped.transform.rotation.y = camera_rotation_inverse.y();
		camera_transform_stamped.transform.rotation.z = camera_rotation_inverse.z();
		
		transform_broadcaster.sendTransform(camera_transform_stamped);

		loop_rate.sleep();
	}

	return 0;
}

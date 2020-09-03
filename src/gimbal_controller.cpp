#include <gimbal_controller.h>

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	tf2::Quaternion rotation;
	tf2::fromMsg(msg->orientation, rotation);
	tf2::Matrix3x3(rotation).getEulerYPR(body_yaw, body_pitch, body_roll);
}

// remove rotation from a pose by rotating it by the inverse of its rotation
void generate_transform_straightened( const geometry_msgs::PoseStamped & _pose_in )
{
	// declare pose variables
	geometry_msgs::PoseStamped pose_out;
	geometry_msgs::PoseStamped pose_in = _pose_in;

	// access the transform broadcaster
	static tf2_ros::TransformBroadcaster transform_broadcaster;

	// declare a message in which to store the new transform
	geometry_msgs::TransformStamped transform_stamped_message;

	// set the transform's header
	transform_stamped_message.header.stamp = ros::Time::now();
	// make the parent frame_id equal to the child's frame_id with '_straightened' appended
	transform_stamped_message.header.frame_id = pose_in.header.frame_id + "_straightened";
	transform_stamped_message.child_frame_id = pose_in.header.frame_id;

	// invert the rotation
	tf2::Quaternion rotation, inverse_rotation;
	tf2::fromMsg(pose_in.pose.orientation, rotation);
	inverse_rotation = rotation.inverse();

	// set the rotation from the parent to the child
	transform_stamped_message.transform.rotation = tf2::toMsg(inverse_rotation);
//	transform_stamped_message.transform.rotation = pose_in.pose.orientation;

	// send the transform
	transform_broadcaster.sendTransform(transform_stamped_message);

	/*
	transform_buffer.setUsingDedicatedThread(true);
	// transform the pose and return it
	return transform_buffer.transform(pose_in, transform_stamped_message.header.frame_id, ros::Duration(0.05));
	*/
}

void gimbal_x_position_callback(const std_msgs::Float64::ConstPtr msg)
{
	gimbal_x_position = msg->data;
}

void gimbal_y_position_callback(const std_msgs::Float64::ConstPtr msg)
{
	gimbal_y_position = msg->data;
}

void whycon_visual_callback(const whycon_ros::MarkerArray::ConstPtr& msg)
{
	// capture the pose
	geometry_msgs::PoseStamped _whycon_camera_pose;	
	_whycon_camera_pose.header.stamp = ros::Time::now();
	_whycon_camera_pose.header.frame_id = "camera_frame_whycon";
	_whycon_camera_pose.pose = msg->markers[0].position;

	// aim the camera
	setpoint_x.data += -setpoint_scalar * _whycon_camera_pose.pose.position.x / _whycon_camera_pose.pose.position.z;
	setpoint_y.data +=  setpoint_scalar * _whycon_camera_pose.pose.position.y / _whycon_camera_pose.pose.position.z;
	
	setpoint_publisher_x.publish(setpoint_x);
	setpoint_publisher_y.publish(setpoint_y);
	idle_state_msg.data = false;

	last_whycon_detection_time = ros::Time::now();

	tf2::Quaternion rotation;
	tf2::fromMsg(_whycon_camera_pose.pose.orientation, rotation);

	double yaw, pitch, roll;
	tf2::Matrix3x3(rotation).getEulerYPR(whycon_yaw, whycon_pitch, whycon_roll);

	// set global and publish
	whycon_camera_pose = _whycon_camera_pose;
	whycon_camera_pose_publisher.publish(whycon_camera_pose);

	// calculate landing pad position using whycon
	geometry_msgs::TransformStamped whycon_camera_transform_stamped;
	whycon_camera_transform_stamped.header.stamp = ros::Time::now();
	whycon_camera_transform_stamped.child_frame_id = "camera_frame_whycon_straightened";
	whycon_camera_transform_stamped.header.frame_id = "body_enu";
	
	// set rotation
	tf2::Quaternion camera_rotation;
	camera_rotation.setRPY(0, 0, gimbal_x_position );
	tf2::Quaternion correction(1, 0, 0, 0);
	tf2::Quaternion total_rotation = camera_rotation * correction;
	tf2::Quaternion inverse_total_rotation = total_rotation.inverse();
	whycon_camera_transform_stamped.transform.rotation = tf2::toMsg(inverse_total_rotation);

	// broadcast transform
	static tf2_ros::TransformBroadcaster transform_broadcaster;
	transform_broadcaster.sendTransform(whycon_camera_transform_stamped);

	try
	{
		generate_transform_straightened(whycon_camera_pose);
	}
	catch( const std::exception & e )
	{
		ROS_WARN("Exception in whycon callback (gimbal_controller)");
		ROS_INFO_STREAM(e.what());
	}

	previous_whycon_camera_pose = whycon_camera_pose;
}

int main(int argc, char **argv)
{
	ROS_INFO("Started gimbal_controller!");

	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	
	// initialize subscribers
	ros::Subscriber whycon_visual_subscriber	= node_handle.subscribe("/whycon_ros/markers",	1000, whycon_visual_callback	);
	ros::Subscriber gimbal_x_position_subscriber	= node_handle.subscribe("/gimbal/x/position",	1000, gimbal_x_position_callback);
	ros::Subscriber gimbal_y_position_subscriber	= node_handle.subscribe("/gimbal/y/position",	1000, gimbal_y_position_callback);
	ros::Subscriber imu_subscriber			= node_handle.subscribe("/imu", 		1000, imu_callback);

	// publisher to set the states of the current gimbal PID control values
	setpoint_publisher_x = node_handle.advertise<std_msgs::Float64>("/gimbal/x/setpoint",	1000);
	setpoint_publisher_y = node_handle.advertise<std_msgs::Float64>("/gimbal/y/setpoint",	1000);
	idle_state_publisher = node_handle.advertise<std_msgs::Bool>("/gimbal/idle_state",	1000);
	idle_state_msg.data  = false;

	// publisher to share the pose of the landing pad within the camera frame
	whycon_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/whycon_pose", 1000);
	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1000);
	yaw_displacement_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/yaw_displacement", 1000);

	// initialize transform broadcaster
	static tf2_ros::TransformBroadcaster transform_broadcaster;

	// initialize gimbal position to forward level
	setpoint_x.data = idle_setpoint_x;
	setpoint_y.data = idle_setpoint_y;
	setpoint_publisher_x.publish(setpoint_x);
	setpoint_publisher_y.publish(setpoint_y);

	transform_buffer.setUsingDedicatedThread(true);

	ros::Time initial = ros::Time::now();
	ros::Duration duration(0.5);
	while( ros::Time::now() - initial < duration )
	{
		idle_state_msg.data = false;
		idle_state_publisher.publish(idle_state_msg);
		
		setpoint_publisher_x.publish(idle_setpoint_x);
		setpoint_publisher_y.publish(idle_setpoint_y);
	}
	idle_state_msg.data = true;
	idle_state_publisher.publish(idle_state_msg);

	// timing
	ros::Duration detection_fail_timeout(2.0);
	ros::Rate loop_rate(70);
	while( ros::ok() )
	{
		ros::spinOnce();

		if( ros::Time::now() - last_whycon_detection_time <= detection_fail_timeout )
		{
			landing_pad_camera_pose = whycon_camera_pose;
			landing_pad_camera_pose_publisher.publish(landing_pad_camera_pose);
		}
		else
		{
			// set the gimbal to the idle attitude
			setpoint_x.data = idle_setpoint_x;
			setpoint_y.data = idle_setpoint_y;
			
			// publish the message
			setpoint_publisher_x.publish(setpoint_x);
			setpoint_publisher_y.publish(setpoint_y);
		}
		
		loop_rate.sleep();
	}

	return 0;
}

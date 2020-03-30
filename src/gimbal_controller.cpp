#include <gimbal_controller.h>

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
		// the landing pad april tag has id 0
		if( _msg->detections[i].id[0] == 0 )
		{
			// get the detection pose
			detection = _msg->detections[i];

			// copy the pose into a PoseStamped message and publish
			geometry_msgs::PoseWithCovarianceStamped buffer = detection.pose;
			geometry_msgs::PoseStamped _apriltag_camera_pose;
			_apriltag_camera_pose.pose = buffer.pose.pose;
			_apriltag_camera_pose.header.stamp = ros::Time::now();
			_apriltag_camera_pose.header.frame_id = "camera_frame_apriltag";

			// extract yaw
			tf2::Quaternion rotation;
			tf2::fromMsg(_apriltag_camera_pose.pose.orientation, rotation);
			tf2::Matrix3x3(rotation).getEulerYPR(landing_pad_yaw, landing_pad_pitch, landing_pad_roll);

			// increment gimbal setpoints, scaling by inverse of z-distance
			setpoint_x.data += -setpoint_scalar * _apriltag_camera_pose.pose.position.x / _apriltag_camera_pose.pose.position.z;
			setpoint_y.data +=  setpoint_scalar * _apriltag_camera_pose.pose.position.y / _apriltag_camera_pose.pose.position.z;

			// publish setpoints and idle state
			setpoint_publisher_x.publish(setpoint_x);
			setpoint_publisher_y.publish(setpoint_y);
			idle_state_msg.data = false;

			// update most recent detection time
			last_apriltag_detection_time = ros::Time::now();
			
			apriltag_camera_pose = _apriltag_camera_pose;
			apriltag_camera_pose_publisher.publish(apriltag_camera_pose);

			// calculate landing pad position using apriltag
			// generate transform
			geometry_msgs::TransformStamped apriltag_camera_transform_stamped;
			apriltag_camera_transform_stamped.header.stamp = ros::Time::now();
			apriltag_camera_transform_stamped.child_frame_id = "camera_frame_apriltag_straightened";
			apriltag_camera_transform_stamped.header.frame_id = "body_enu";

			apriltag_camera_transform_stamped.transform.translation.y = -0.75;

			// set rotation
			tf2::Quaternion camera_rotation, inverse_camera_rotation;
			camera_rotation.setRPY(0, 0, - gimbal_x_position + landing_pad_yaw);
			inverse_camera_rotation = camera_rotation.inverse();
			apriltag_camera_transform_stamped.transform.rotation = tf2::toMsg(inverse_camera_rotation);

			// broadcast transform
			static tf2_ros::TransformBroadcaster transform_broadcaster;
			transform_broadcaster.sendTransform(apriltag_camera_transform_stamped);
		}
	}
}

void whycon_visual_callback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
	// capture the pose
	geometry_msgs::PoseStamped _whycon_camera_pose;	
	_whycon_camera_pose.header.stamp = ros::Time::now();
	_whycon_camera_pose.header.frame_id = "camera_frame_whycon";
	_whycon_camera_pose.pose = msg->markers[0].pose;

	if( ros::Time::now() - last_apriltag_detection_time >= detection_timeout )
	{
		// this sort of thing seems to work the best using PID systems on the x and y positions
		setpoint_x.data += -setpoint_scalar * _whycon_camera_pose.pose.position.x / _whycon_camera_pose.pose.position.z;
		setpoint_y.data +=  setpoint_scalar * _whycon_camera_pose.pose.position.y / _whycon_camera_pose.pose.position.z;
		
		setpoint_publisher_x.publish(setpoint_x);
		setpoint_publisher_y.publish(setpoint_y);
		idle_state_msg.data = false;

	}

	last_whycon_detection_time = ros::Time::now();

	double yaw, pitch, roll;
	tf2::Quaternion rotation;
	tf2::fromMsg(_whycon_camera_pose.pose.orientation, rotation);
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
//		camera_rotation.setRPY(0, 0, - gimbal_x_position + whycon_yaw);
	tf2::Quaternion camera_rotation;
	camera_rotation.setRPY(0, 0, gimbal_x_position );
//		tf2::Quaternion correction(0, 0, 1, 0);
	tf2::Quaternion correction(1, 0, 0, 0);
	tf2::Quaternion total_rotation = camera_rotation * correction;
	tf2::Quaternion inverse_total_rotation = total_rotation.inverse();
	whycon_camera_transform_stamped.transform.rotation = tf2::toMsg(inverse_total_rotation);

	// broadcast transform
	static tf2_ros::TransformBroadcaster transform_broadcaster;
	transform_broadcaster.sendTransform(whycon_camera_transform_stamped);
}

int main(int argc, char **argv)
{
	ROS_INFO("Started gimbal_controller!");

	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	
	// initialize subscribers
	ros::Subscriber whycon_visual_subscriber	= node_handle.subscribe("/whycon_ros/visual",	1000, whycon_visual_callback	);
	ros::Subscriber apriltag_visual_subscriber	= node_handle.subscribe("/tag_detections",	1000, apriltag_visual_callback	);
	ros::Subscriber gimbal_x_position_subscriber	= node_handle.subscribe("/gimbal/x/position",	1000, gimbal_x_position_callback);
	ros::Subscriber gimbal_y_position_subscriber	= node_handle.subscribe("/gimbal/y/position",	1000, gimbal_y_position_callback);
	
	// publisher to set the states of the current gimbal PID control values
	setpoint_publisher_x = node_handle.advertise<std_msgs::Float64>("/gimbal/x/setpoint",	1000);
	setpoint_publisher_y = node_handle.advertise<std_msgs::Float64>("/gimbal/y/setpoint",	1000);
	idle_state_publisher = node_handle.advertise<std_msgs::Bool>("/gimbal/idle_state",	1000);
	idle_state_msg.data  = false;

	// publisher to share the pose of the landing pad within the camera frame
	whycon_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/whycon_pose", 1000);
	apriltag_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/apriltag_pose", 1000);
	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1000);

	// initialize transform broadcaster
	static tf2_ros::TransformBroadcaster transform_broadcaster;

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

		last_detection_time = last_apriltag_detection_time;
		
		landing_pad_camera_pose = apriltag_camera_pose;
		if( ros::Time::now() - last_apriltag_detection_time >= detection_timeout )
//		if( last_whycon_detection_time > last_apriltag_detection_time )
		{
			last_detection_time = last_whycon_detection_time;
			landing_pad_camera_pose = whycon_camera_pose;
		}

		landing_pad_camera_pose_publisher.publish(landing_pad_camera_pose);

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

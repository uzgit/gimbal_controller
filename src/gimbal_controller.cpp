#include <gimbal_controller.h>

std_msgs::Float64 to_msg(const double data)
{
	std_msgs::Float64 message;
	message.data = data;
	return message;
}

double interpolate(double input, double input_min, double input_max, double output_min, double output_max)
{
	double result = output_min + (input - input_min) * (output_max - output_min) / (input_max - input_min);

	return result;
}

void calculate_camera_angles()
{
	tilt_angle = interpolate(tilt_pwm, PWM_MIN, PWM_MAX, tilt_min, tilt_max);
	pan_angle  = interpolate(pan_pwm,  PWM_MIN, PWM_MAX, pan_min,  pan_max );

	camera_tilt_publisher.publish(to_msg(tilt_angle));
	camera_pan_publisher.publish(to_msg(pan_angle));
}

void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
	state = *msg;
}

// normalize a pixel position from [0, range_max] into [-1.0, 1.0] for PID control
double normalize_pixel_position(double pixel_position, double range_max)
{
	double range  = range_max / 2.0;
	double result = (pixel_position - range) / range;

	return result;
}

double normalize(double data, double minimum, double maximum)
{
	double average = (minimum + maximum) / 2.0;
	double result  = (data - average) / average;

	return result;
}

int control_effort_to_pwm_signal(double control_effort)
{
	int result = 1500 + (500 * control_effort);

	return result;
}

void send_rc_control(int tilt_control_effort, int pan_control_effort)
{
	override_rc_in_message.channels[TILT_CHANNEL - 1] = tilt_control_effort;
	override_rc_in_message.channels[PAN_CHANNEL - 1] =  pan_control_effort;

	override_rc_in_publisher.publish(override_rc_in_message);
}

void camera_control_effort_x_callback(const std_msgs::Float64::ConstPtr& msg)
{
	camera_pid_control_effort_x.data = msg->data;
	pan_pwm = control_effort_to_pwm_signal(camera_pid_control_effort_x.data);
}

void camera_control_effort_y_callback(const std_msgs::Float64::ConstPtr& msg)
{
	camera_pid_control_effort_y.data = msg->data;
	tilt_pwm = control_effort_to_pwm_signal(camera_pid_control_effort_y.data);
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

void whycon_visual_callback(const whycon_ros::MarkerArray::ConstPtr& msg)
{
	last_whycon_detection_time = ros::Time::now();

	// capture the pose
	geometry_msgs::PoseStamped _whycon_camera_pose;	
	_whycon_camera_pose.header.stamp = ros::Time::now();
	_whycon_camera_pose.header.frame_id = "camera_frame_whycon";
	_whycon_camera_pose.pose = msg->markers[0].position;

	// publish the whycon/whycode's normalized [-1.0,1.0] pixel positions for aiming the camera
	landing_pad_pixel_position_x.data = normalize_pixel_position( msg->markers[0].u, camera_pixel_width );
	landing_pad_pixel_position_y.data = normalize_pixel_position( msg->markers[0].v, camera_pixel_height );
	landing_pad_pixel_position_x_publisher.publish(landing_pad_pixel_position_x);
	landing_pad_pixel_position_y_publisher.publish(landing_pad_pixel_position_y);

	// publish setpoints of 0 (centered)
	camera_pid_setpoint_x_publisher.publish(camera_pid_setpoint_x);
	camera_pid_setpoint_y_publisher.publish(camera_pid_setpoint_y);

	tf2::Quaternion rotation, current_inverse_rotation, previous_rotation, difference;

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
	camera_rotation.setRPY(0, 0, pan_angle );
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
}

int main(int argc, char **argv)
{
	// say hello
	ROS_INFO("Started gimbal_controller!");

	// initialize node
	ros::init(argc, argv, "gimbal_controller");
	ros::NodeHandle node_handle;
	
	std_msgs::Bool std_msgs_false;
	std_msgs_false.data = false;
	std_msgs::Bool std_msgs_true;
	std_msgs_true.data = true;

	// initialize subscribers
	ros::Subscriber whycon_visual_subscriber	= node_handle.subscribe("/whycon_ros/markers",	1000, whycon_visual_callback	);
	ros::Subscriber pid_x_control_effort_subscriber = node_handle.subscribe("/pid/camera/control_effort/x", 1000, camera_control_effort_x_callback);
	ros::Subscriber pid_y_control_effort_subscriber = node_handle.subscribe("/pid/camera/control_effort/y", 1000, camera_control_effort_y_callback);
	ros::Subscriber state_subscriber		= node_handle.subscribe("/mavros/state", 1000, state_callback);

	// initialize publishers
	landing_pad_pixel_position_x_publisher	= node_handle.advertise<std_msgs::Float64>("/landing_pad/pixel_position/x", 1000);
	landing_pad_pixel_position_y_publisher	= node_handle.advertise<std_msgs::Float64>("/landing_pad/pixel_position/y", 1000);
	camera_pid_enable_x_publisher		= node_handle.advertise<std_msgs::Bool>("/pid/camera/x/enable", 1000);
	camera_pid_enable_y_publisher		= node_handle.advertise<std_msgs::Bool>("/pid/camera/y/enable", 1000);
	whycon_camera_pose_publisher		= node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/whycon_pose", 1000);
	landing_pad_camera_pose_publisher	= node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1000);
	camera_pid_setpoint_x_publisher		= node_handle.advertise<std_msgs::Float64>("/pid/camera/setpoint/x", 1000);
	camera_pid_setpoint_y_publisher		= node_handle.advertise<std_msgs::Float64>("/pid/camera/setpoint/y", 1000);
	camera_tilt_publisher			= node_handle.advertise<std_msgs::Float64>("/camera/tilt", 1000);
	camera_pan_publisher			= node_handle.advertise<std_msgs::Float64>("/camera/pan", 1000);
	override_rc_in_publisher		= node_handle.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1000);
	
	// initialize all rc channel overrides to 0 so they will be ignored
	for(int channel = 0; channel < 8; channel ++)
	{
		override_rc_in_message.channels[channel] = 0;
	}

	// initialize transform broadcaster
	static tf2_ros::TransformBroadcaster transform_broadcaster;

	// initialize camera setpoints
	camera_pid_setpoint_x.data = 0;
	camera_pid_setpoint_y.data = 0;

	// for tf2
	transform_buffer.setUsingDedicatedThread(true);

	// 2 second timeout before ceding control of the gimbal
	ros::Duration detection_fail_timeout(2.0);
	
	// ********************************************************************************************
	ros::Rate loop_rate(50);
	while( ros::ok() )
	{
		// callbacks
		ros::spinOnce();

		// check for recent detection
		if( ros::Time::now() - last_whycon_detection_time <= detection_fail_timeout )
		{
			// publish the whycon pose as the landing pad pose since it is the only marker
			landing_pad_camera_pose = whycon_camera_pose;
			landing_pad_camera_pose_publisher.publish(landing_pad_camera_pose);

			// send PWM signals to MAVROS to control the gimbal
			send_rc_control(tilt_pwm, pan_pwm);

			// calculate physical camera angles from given PWM signals
			calculate_camera_angles();
		}
	
		loop_rate.sleep();
	}
	// ********************************************************************************************
	
	return 0;
}

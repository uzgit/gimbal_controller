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

	camera_tilt_publisher.publish(to_msg(tilt_angle));
}

void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
	state = *msg;
}

int control_effort_to_pwm_signal(double control_effort)
{
	int result = 1500 + (500 * control_effort);

	return result;
}

void send_rc_control(int tilt_control_effort)
{
	override_rc_in_message.channels[TILT_CHANNEL - 1] = tilt_control_effort;

	override_rc_in_publisher.publish(override_rc_in_message);
}

void camera_control_effort_y_callback(const std_msgs::Float64::ConstPtr& msg)
{
	camera_pid_control_effort_y.data = msg->data;
	tilt_pwm = control_effort_to_pwm_signal(camera_pid_control_effort_y.data);
}

void apriltag3_visual_callback( const apriltag_ros::AprilTagDetectionArray& msg )
{
	int i = 0;
	while( i < msg.detections.size() && msg.detections[i].name != "landing_pad" ) i ++;

	if( i < msg.detections.size() )
	{
		// get the normalized pixel positions
		landing_pad_pixel_position_x.data = msg.detections[i].c_normalized[0];
		landing_pad_pixel_position_y.data = msg.detections[i].c_normalized[1];

		// publish the normalized pixel positions to the pid controllers
		landing_pad_pixel_position_x_publisher.publish( landing_pad_pixel_position_x );
		landing_pad_pixel_position_y_publisher.publish( landing_pad_pixel_position_y );

		// publish pid setpoints
		camera_pid_setpoint_x_publisher.publish( camera_pid_setpoint_x );
		camera_pid_setpoint_y_publisher.publish( camera_pid_setpoint_y );

		last_apriltag_detection_time = ros::Time::now();
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
//	ros::Subscriber whycon_visual_subscriber	= node_handle.subscribe("/whycon_ros/markers",	1000, whycon_visual_callback	);
	ros::Subscriber apriltag3_subscriber		= node_handle.subscribe("/tag_detections", 		1000, apriltag3_visual_callback);
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
	override_rc_in_publisher		= node_handle.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1000);
	
	// initialize all rc channel overrides to 0 so they will be ignored
	for(int channel = 0; channel < 8; channel ++)
	{
		override_rc_in_message.channels[channel] = 0;
	}

	// initialize camera setpoints
	camera_pid_setpoint_x.data = 0;
	camera_pid_setpoint_y.data = 0;

	for(int i = 0; i < 100; i ++)
	{
		send_rc_control(1750);
	}

	// 2 second timeout before ceding control of the gimbal
	ros::Duration detection_fail_timeout(2.0);
	bool detection_failure = true;

	// ********************************************************************************************
	ros::Rate loop_rate(50);
	while( ros::ok() )
	{
		ROS_INFO("in gimbal_controller main loop!");

		// callbacks
		ros::spinOnce();

		// check for recent detection
		if( ros::Time::now() - last_apriltag_detection_time <= detection_fail_timeout )
		{
			// send PWM signals to MAVROS to control the gimbal
			send_rc_control(tilt_pwm);

			// calculate physical camera angles from given PWM signals
			calculate_camera_angles();

			detection_failure = false;
		}
		else if( ! detection_failure )
		{
			send_rc_control(1500);

			detection_failure = true;
		}
	
		loop_rate.sleep();
	}
	// ********************************************************************************************
	
	return 0;
}

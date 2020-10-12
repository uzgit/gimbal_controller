#include <whycon_data_collection.h>

// remove rotation from a pose by rotating it by the inverse of its rotation
geometry_msgs::PoseStamped straighten_pose( const geometry_msgs::PoseStamped & _pose_in )
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

	// transform the pose and return it
//	return transform_buffer.transform(pose_in, transform_stamped_message.header.frame_id, ros::Duration(0.05));

	// for the specific whycon_data_collection module only, since we do not need to broadcast transforms but merely execute them	
	tf2::doTransform(pose_in, pose_out, transform_stamped_message);
	return pose_out;
}

void whycon_visual_callback(const whycon::MarkerArray::ConstPtr& msg)
{
	// capture the pose
	geometry_msgs::PoseStamped _whycon_camera_pose, straightened_pose;
	_whycon_camera_pose.header.stamp = ros::Time::now();
	_whycon_camera_pose.header.frame_id = "camera_frame_whycon";
	_whycon_camera_pose.pose = msg->markers[0].position;

	ROS_INFO_STREAM(_whycon_camera_pose);

	try
	{
		straightened_pose = straighten_pose(_whycon_camera_pose);
//		ROS_INFO_STREAM(pose_straightened);
	}
	catch( tf2::TransformException &exception )
	{
		ROS_WARN("Transform exception in landing_controller main loop: %s", exception.what());
	}

	ROS_INFO_STREAM(msg->markers[0].id);

	output_file << ros::Time::now()
		    << ","
		    << (int32_t)msg->markers[0].id
		    << ","
		    << msg->markers[0].u
		    << ","
		    << msg->markers[0].v
		    << ","
		    << msg->markers[0].angle
		    << ","
		    << _whycon_camera_pose.pose.position.x
		    << ","
		    << _whycon_camera_pose.pose.position.y
		    << ","
		    << _whycon_camera_pose.pose.position.z
		    << ","
		    << msg->markers[0].rotation.x
		    << ","
		    << msg->markers[0].rotation.y
		    << ","
		    << msg->markers[0].rotation.z
		    << ","
		    << _whycon_camera_pose.pose.orientation.x
		    << ","
		    << _whycon_camera_pose.pose.orientation.y
		    << ","
		    << _whycon_camera_pose.pose.orientation.z
		    << ","
		    << _whycon_camera_pose.pose.orientation.w
		    << ","
		    << straightened_pose.pose.position.x
		    << ","
		    << straightened_pose.pose.position.y
		    << ","
		    << straightened_pose.pose.position.z
		    << ","
		    << straightened_pose.pose.orientation.x
		    << ","
		    << straightened_pose.pose.orientation.y
		    << ","
		    << straightened_pose.pose.orientation.z
		    << ","
		    << straightened_pose.pose.orientation.w
		    << std::endl;

}

void sigint_handler(int signum)
{
	output_file.close();

	exit(signum);
}

int main(int argc, char **argv)
{
	ROS_INFO("Started whycon_data_collection node!");

	ros::init(argc, argv, "whycon_data_collection");
	ros::NodeHandle node_handle;

	// for handling output file
	signal(SIGINT, sigint_handler);
	output_file.open("/home/joshua/Documents/whycon_data.csv");

	// write output file header
	output_file << "time"
		    << ","
		    << "id"
		    << ","
		    << "u"
		    << ","
		    << "v"
		    << ","
		    << "angle"
		    << ","
		    << "position_x"
		    << ","
		    << "position_y"
		    << ","
		    << "position_z"
		    << ","
		    << "roll"
		    << ","
		    << "pitch"
		    << ","
		    << "yaw"
		    << ","
		    << "orientation_x"
		    << ","
		    << "orientation_y"
		    << ","
		    << "orientation_z"
		    << ","
		    << "orientation_w"
		    << ","
		    << "straightened_position_x"
		    << ","
		    << "straightened_position_y"
		    << ","
		    << "straightened_position_z"
		    << ","
		    << "straightened_orientation_x"
		    << ","
		    << "straightened_orientation_y"
		    << ","
		    << "straightened_orientation_z"
		    << ","
		    << "straightened_orientation_w"
		    << std::endl;
		    
	// initialize subscribers
	ros::Subscriber whycon_visual_subscriber	= node_handle.subscribe("/whycon/markers",	1000, whycon_visual_callback	);

	// publisher to share the pose of the landing pad within the camera frame
	whycon_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/whycon_pose", 1000);
	landing_pad_camera_pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("/landing_pad/camera_pose", 1000);
	yaw_displacement_publisher = node_handle.advertise<std_msgs::Float64>("/landing_pad/yaw_displacement", 1000);

	// initialize transform broadcaster
	static tf2_ros::TransformBroadcaster transform_broadcaster;

	transform_buffer.setUsingDedicatedThread(true);

	// timing
	ros::Duration detection_fail_timeout(2.0);
	ros::Rate loop_rate(70);
	while( ros::ok() )
	{
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}

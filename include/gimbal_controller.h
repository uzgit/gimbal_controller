#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

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

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <cmath>
#include <ctime>

int landing_pad_id[2] = {1, 2};
//int apriltag_offset = 0.75;
int apriltag_offset = 1.75;

double idle_setpoint_x = 0.0;
double idle_setpoint_y = 0.4;

double gimbal_x_position = std::nan("1");
double gimbal_y_position = std::nan("1");
double landing_pad_yaw;
double landing_pad_pitch;
double landing_pad_roll;
double whycon_yaw;
double whycon_pitch;
double whycon_roll;

std_msgs::Float64 setpoint_x;
std_msgs::Float64 setpoint_y;
std_msgs::Bool idle_state_msg;
double setpoint_scalar = 0.25;

ros::Publisher setpoint_publisher_x;
ros::Publisher setpoint_publisher_y;
ros::Publisher idle_state_publisher;
ros::Publisher landing_pad_camera_pose_publisher;
ros::Publisher whycon_camera_pose_publisher;
ros::Publisher apriltag_camera_pose_publisher;

ros::Duration detection_timeout(2);
ros::Time last_detection_time(0);
ros::Time last_apriltag_detection_time(0);
ros::Time last_whycon_detection_time(0);

tf2_ros::Buffer transform_buffer;

geometry_msgs::PoseStamped whycon_camera_pose;
geometry_msgs::PoseStamped apriltag_camera_pose;
geometry_msgs::PoseStamped landing_pad_camera_pose;

void gimbal_x_position_callback( const std_msgs::Float64::ConstPtr );
void gimbal_y_position_callback( const std_msgs::Float64::ConstPtr );
void apriltag_visual_callback( const apriltag_ros::AprilTagDetectionArray::ConstPtr& );
void whycon_visual_callback( const visualization_msgs::MarkerArray::ConstPtr& );
geometry_msgs::PoseStamped straighten_pose( const geometry_msgs::PoseStamped& );

#endif

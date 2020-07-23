#ifndef GIMBAL_CONTROLLER_H
#define GIMBAL_CONTROLLER_H

#include <cstdlib>
#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <whycon_ros/MarkerArray.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

#include <cmath>
#include <ctime>
#include <algorithm>

int landing_pad_id[2] = {1, 2};
double apriltag_offset = 0.75;
//int apriltag_offset = 1;

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

double body_pitch;
double body_roll;
double body_yaw;

std_msgs::Float64 landing_pad_pixel_position_x;
std_msgs::Float64 landing_pad_pixel_position_y;
std_msgs::Float64 camera_pid_control_effort_x;
std_msgs::Float64 camera_pid_control_effort_y;
std_msgs::Float64 camera_pid_setpoint_x;
std_msgs::Float64 camera_pid_setpoint_y;
std_msgs::Float64 setpoint_x;
std_msgs::Float64 setpoint_y;
std_msgs::Bool idle_state_msg;
std_msgs::Bool camera_pid_x_enable;
std_msgs::Bool camera_pid_y_enable;
double setpoint_scalar = 0.75;

// these may actually change through the course of execution so they need to be actual variables
double camera_pixel_height = 640;
double camera_pixel_width  = 480;

int tilt_idle_pwm = 1350;
int pan_idle_pwm  = 1500;
int tilt_pwm = tilt_idle_pwm;
int pan_pwm  = pan_idle_pwm;

#define PWM_MIN 1100
#define PWM_MAX 1900
#define TILT_CHANNEL 7
#define PAN_CHANNEL  8
//int pan_channel  = 6;
//int tilt_channel = 8;
double pan_max  =  0.523599; // -30 deg
double pan_min  = -0.523599; //  30 deg
double tilt_max =  0.174533; //  10 deg
double tilt_min = -1.5708;   // -90 deg

double pan_angle = 0;
double tilt_angle = 0;

mavros_msgs::OverrideRCIn override_rc_in_message;
mavros_msgs::State state;

ros::Publisher setpoint_publisher_x;
ros::Publisher setpoint_publisher_y;
ros::Publisher landing_pad_pixel_position_x_publisher;
ros::Publisher landing_pad_pixel_position_y_publisher;
ros::Publisher camera_pid_enable_x_publisher;
ros::Publisher camera_pid_enable_y_publisher;
ros::Publisher camera_pid_setpoint_x_publisher;
ros::Publisher camera_pid_setpoint_y_publisher;
ros::Publisher idle_state_publisher;
ros::Publisher landing_pad_camera_pose_publisher;
ros::Publisher whycon_camera_pose_publisher;
ros::Publisher apriltag_camera_pose_publisher;
ros::Publisher yaw_displacement_publisher;
ros::Publisher landing_pad_relative_pose_publisher;
ros::Publisher vo_publisher;
ros::Publisher override_rc_in_publisher;
ros::Publisher camera_pid_x_enable_publisher;
ros::Publisher camera_pid_y_enable_publisher;
ros::Publisher camera_pan_publisher;
ros::Publisher camera_tilt_publisher;

ros::Duration detection_timeout(2);
ros::Time last_detection_time(0);
ros::Time last_apriltag_detection_time(0);
ros::Time last_whycon_detection_time(0);

tf2_ros::Buffer transform_buffer;

geometry_msgs::PoseStamped whycon_camera_pose;
geometry_msgs::PoseStamped previous_whycon_camera_pose;
geometry_msgs::PoseStamped apriltag_camera_pose;
geometry_msgs::PoseStamped landing_pad_camera_pose;
geometry_msgs::PoseStamped landing_pad_relative_pose;

void gimbal_x_position_callback( const std_msgs::Float64::ConstPtr );
void gimbal_y_position_callback( const std_msgs::Float64::ConstPtr );
void apriltag_visual_callback( const apriltag_ros::AprilTagDetectionArray::ConstPtr& );
void whycon_visual_callback( const whycon_ros::MarkerArray::ConstPtr& );

geometry_msgs::PoseStamped straighten_pose( const geometry_msgs::PoseStamped& );

nav_msgs::Odometry apriltag_odometry_message;
nav_msgs::Odometry whycon_odometry_message;

#endif

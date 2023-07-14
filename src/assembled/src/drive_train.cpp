#include "drive_train.h"
#include <iostream>
#include <tuple>
// ros
#include <ros/ros.h>


void DifferentialDrive::init(){
    const std::string param_prefix = "/assembled/differential_drive/";
    ros::param::param<float>(param_prefix + "wheel_max_speed", max_speed, 500.0);
    ros::param::param<float>(param_prefix + "wheel_min_speed", min_speed, 200.0);
    ros::param::param<float>(param_prefix + "wheel_radius", wheel_radius, 0.04);
    ros::param::param<float>(param_prefix + "wheel_separation", wheel_seperation, 0.0182);
    
    ROS_INFO_STREAM("wheel_max_speed: " << max_speed);  
}
DifferentialDrive::DifferentialDrive(ros::NodeHandle&nh):
DriveTrain(nh),
linear_x(0), angular_z(0), v_r(0), v_l(0), vx(0), vth(0)
{
    init();
    rw_vel_pub = nh.advertise<std_msgs::Float64>("/rwheel_vtarget", 1);
    lw_vel_pub = nh.advertise<std_msgs::Float64>("/lwheel_vtarget", 1);
}
void DifferentialDrive::ComputeWheelSpeeds()
{
    ROS_INFO("v w :");
    ROS_INFO("v: %f, w: %f", linear_x, angular_z);
    // 
    v_r = -((2 * linear_x) + (angular_z * wheel_seperation)) / (2 * wheel_radius);
    v_l = ((2 * linear_x) - (angular_z * wheel_seperation)) / (2 * wheel_radius);

    ROS_INFO("speeds rads/sec:");
    ROS_INFO("v_r: %f, v_l: %f", v_r, v_l);
    v_r *=  steps_per_rev/( 2 * 3.1516);
    v_l *=  steps_per_rev/( 2 * 3.1516);
    ROS_INFO("speeds before filtering:");
    ROS_INFO("v_r: %f, v_l: %f", v_r, v_l);
}

void DifferentialDrive::brake(){
    v_r = 0.0;
    v_l = 0.0;
}

void DifferentialDrive::publish(){
    // make msgs from speeds
    std_msgs::Float64 rw_msg;
    std_msgs::Float64 lw_msg;
    rw_msg.data = v_r;
    lw_msg.data = v_l;
    // publish
    rw_vel_pub.publish(rw_msg);
    lw_vel_pub.publish(lw_msg);
}
// apply speed limits
void DifferentialDrive::applyWheelSpeedsLimits(){
// minimum angular speed in rad/s  = 2.94
    if (v_r > max_speed) v_r = max_speed;
    else if (v_r < -max_speed) v_r = -max_speed;

    if (v_l > max_speed) v_l = max_speed;
    else if (v_l < -max_speed) v_l = -max_speed;

    if (v_r < min_speed && v_r  > 0) v_r = 0;
    else if (v_r > -min_speed && v_r < 0) v_r = 0;

    if (v_l < min_speed && v_l > 0) v_l = 0;
    else if (v_l > -min_speed && v_l < 0) v_l = 0;

    ROS_INFO("speeds after filtering:");
    ROS_INFO("v_r: %f, v_l: %f", v_r, v_l);

    // TODO: apply state machine to enable turning in place and other maneuvers such as turning around one wheel

} 

void DifferentialDrive::computeOdom(){
    vx = (-v_r + v_l) / 2 * (2 * 3.1416 * wheel_radius / steps_per_rev); // steps/s *  (circumf/ rev / steps/rev) =m/s 
    vth = (-v_r - v_l) * (2 * 3.1416 * wheel_radius / steps_per_rev) / wheel_seperation; // rad/s
}
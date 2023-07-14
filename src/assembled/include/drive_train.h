#ifndef DRIVE_TRAIN_H
#define DRIVE_TRAIN_H

#include <iostream>
#include <tuple>
// ros
#include <ros/ros.h>
#include <std_msgs/Float64.h>
// urdf 
#include <urdf/urdfdom_compatibility.h>
#include <urdf_parser/urdf_parser.h>

class DriveTrain
{
private:
    /* data */

    ros::NodeHandle nh;

public:
    DriveTrain(ros::NodeHandle& nh):nh(nh){}
    ~DriveTrain(){}
    virtual void init() = 0;
    virtual void ComputeWheelSpeeds() = 0;
    virtual void brake() = 0;
};

class DifferentialDrive : public DriveTrain
{
private:
    /* data */
    float wheel_radius{0.04};
    float wheel_seperation{0.182};
    // inputs
    float linear_x,  angular_z;
    // outputs steps/s
    float v_r, v_l;
    // odometry from actual wheel speeds after applying limits
    float vx, vth;

    // awheel_seperation speeds are steps/s
    float max_speed=500;
    float min_speed=200;
    const int steps_per_rev=200;
    // ros
    ros::Publisher rw_vel_pub;
    ros::Publisher lw_vel_pub;
    
protected:
    void init();
    void computeOdom();
    void applyWheelSpeedsLimits();

public:
    DifferentialDrive (ros::NodeHandle& nh);
    ~DifferentialDrive() = default;
    void ComputeWheelSpeeds () override;
    void brake() override;
    void update(const float &x, const float& z){
        linear_x = x;
        angular_z = z;
        ComputeWheelSpeeds();
        applyWheelSpeedsLimits();
        computeOdom();
    }
    // publish wheel speeds to
    void publish();

    std::pair<float, float> getWheelSpeeds(){
        return std::make_pair(v_r, v_l);
    }

    // get odom
    std::pair<float, float> getOdom(){
        ROS_INFO_STREAM("Odometry: vx: " << vx << " vth: " << vth);
        return std::make_pair(vx, vth);
    }
};

#endif // DRIVE_TRAIN_H

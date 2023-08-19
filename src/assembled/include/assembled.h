#ifndef ASSEMBLED_H
#define ASSEMBLED_H
#include <vector>
#include "ros/ros.h"
#include"std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include "drive_train.h"
#include "sensor2pc_manager.h"
#include <memory>
#include "sensor.h"


class Assembled
{
public:
    Assembled(std::string name);
    ~Assembled() = default;
    // passable function 
    void performRecoveryRoutine(Sensor& sensor);
    enum class State {IDLE, MOVING, STOPPED, ERROR, REVERSE};
    void changeState(State new_state);

  protected:
    void Init();
    void velCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
    void publishOdometry();
    State state{State::IDLE};
  private:
    ros::NodeHandle nh;
    // velocity sub
    

    ros::Subscriber vel_sub;
    std::unique_ptr<DifferentialDrive> diff_drive_ptr;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Timer timer;
    ros::Time current_time, last_time;
    Timeout timeout;
    double v{0}, w{0}, x{0}, y{0}, th{0};
    double vx{0}, vy{0}, vth{0};
    std::vector<std::unique_ptr<Sensor>> sensors;
    std::string name;

    // const float max_linear_x= max_speed * (2*3.1416/steps_per_rev) * wr/1000;
    // const float min_linear_x= min_speed * (2*3.1416/steps_per_rev) * wr/1000;

};

#endif // ASSEMBLED_H

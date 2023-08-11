#include"assembled.h"
Assembled::Assembled(std::string name):
           name(name),  
           nh("~"),
           vel_sub(nh.subscribe("/cmd_vel", 0.2, &Assembled::velCallback, this)),
           odom_pub(nh.advertise<nav_msgs::Odometry>("odom", 50)),
           v(0), w(0), x(0), y(0), th(0), vx(0), vy(0), vth(0)
{
    Init();
}

void Assembled::Init(){
    // based on assembled/config/assembled_param.yaml
     const std::string param_prefix = "/assembled/";
     const std::string bumper_locations_prefix = "/bumpers_locations/";
    // get params
    const float wr = ros::param::param<float>(param_prefix + "timer_period", 0.3);
    timeout = Timeout(ros::Duration(wr*2));
    timer = nh.createTimer(ros::Duration(ros::Duration(wr)), &Assembled:: timerCallback, this);
    diff_drive_ptr = std::make_unique<DifferentialDrive>(name, nh);
    // print params
    ROS_INFO_STREAM("timer_period: " << wr);

    const std::string bumper_prefix = param_prefix + "bumpers/";
    // add sensors from params
    std::vector<std::string> bumper_names;
    
    if (ros::param::has(bumper_prefix))
    {
        nh.getParam(bumper_prefix, bumper_names);
        ROS_INFO_STREAM("bumper size: " << bumper_names.size());
         for (auto bname : bumper_names)
    {
        ROS_INFO_STREAM("bumper_name: " << bname);
        const std::string bprefix = bumper_locations_prefix + bname + "/";
        if (ros::param::has(bprefix)){
            const float x = ros::param::param<float>(bprefix + "x", 0.0);
            const float y = ros::param::param<float>(bprefix + "y", 0.0);
            const float z = ros::param::param<float>(bprefix + "z", 0.0);
            ROS_INFO_STREAM("bumper_name: " << bname << ". x: " << x << " y: " << y << " z: " << z);
            sensors.push_back(std::make_unique<bumper>(nh, this->name + "/" + bname, x, y, z));
 
        }
        else
        {
            ROS_INFO_STREAM("could not find bumper location for: " << bname << "putting zeros.");
        }
   }

    }
    else
    {
        ROS_INFO_STREAM("bumper_prefix does not exist");
    }

   
}

bool Assembled::checkSensors()
{
    bool ret = true;
    for (auto &s : sensors)
    {
        if (s->isTriggered())
        {
            ROS_WARN_STREAM("Sensor triggered: " << s->getName());
            ret = false;
            auto sensor2pc_manager = Sensor2PcManager::getInstance(nh, "base_link", "sensors" );
            float x, y, z;
            std::tie(x, y, z) = s->getCoordinates();
            sensor2pc_manager->publish( x, y, z );
        }
    }
    return ret;
}
void Assembled::velCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    v = msg->linear.x;
    w = msg->angular.z;

    timeout.Update();
}

void Assembled::timerCallback(const ros::TimerEvent &event)
{
    // main robot loop
    if (diff_drive_ptr == nullptr)
    {
        ROS_ERROR("diff_drive_ptr is null");
        throw std::runtime_error("diff_drive_ptr is null");
        return;
    }
    current_time = ros::Time::now();
    // get odometry
    std::tie(vx, vth)  = diff_drive_ptr->getOdom();
     // compute odometry
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    // publish odometry
    x += delta_x;
    y += delta_y;
    th += delta_th;
    //  publish odom and transform
    publishOdometry();
    
    // publish new wheel speeds
    // if we didn't receive new commands for a while, we'll stop
    // if any of the sensors are triggered, we'll stop (TO-DO later do recovery behavior)
    if (timeout.isTimedOut() && (v != 0.0 || w != 0.0))
    {
        diff_drive_ptr->brake();
        v = 0.0; w = 0.0;
        ROS_WARN("drive train timeout");
    }
    else if (!checkSensors() && (v != 0.0 || w != 0.0))
    {
        diff_drive_ptr->brake();
        v = 0.0; w = 0.0;
        ROS_WARN("sensor triggered and moving");
    }
    else
    {
        diff_drive_ptr->update(v, w);
    }
    diff_drive_ptr->publish();
    last_time = current_time;
}

void Assembled::publishOdometry()
{
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    auto odom_quat = tf::createQuaternionMsgFromYaw(th);
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation =  odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);


    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

}


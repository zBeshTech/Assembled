#ifndef SENSOR2PC_MANAGER_H
#define SENSOR2PC_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <memory>
#include <tf2_ros/transform_listener.h>
class Sensor2PcManager
{
  protected:
  // implement this as a singleton
  static bool is_initialized;
  static Sensor2PcManager* singleton;

  std::unique_ptr<sensor_msgs::PointCloud2Modifier> modifier;

  ros::Publisher pc_pub;
  sensor_msgs::PointCloud2 pc;
  std::string topic_name;
  ros::NodeHandle* nh;
  void initPc(std::string frame_id);
  void addPoint(float x, float y, float z);
  void publishPc();
  Sensor2PcManager( std::string topic_name, std::string frame_id, ros::NodeHandle& nh);

public:
    void publish(float x, float y, float z);
    static Sensor2PcManager* getInstance(ros::NodeHandle& nh, std::string frame_id, std::string topic_name );

    Sensor2PcManager(Sensor2PcManager&&) = delete;
    Sensor2PcManager& operator=(Sensor2PcManager&&) = delete;
    void operator=(Sensor2PcManager const&) = delete;
    Sensor2PcManager(Sensor2PcManager const&) = delete;

    ~Sensor2PcManager();
};


#endif // SENSOR2PC_MANAGER_H
#include "sensor2pc_manager.h"

// static members
bool Sensor2PcManager::is_initialized{false};
Sensor2PcManager* Sensor2PcManager::singleton = nullptr;

Sensor2PcManager::Sensor2PcManager(std::string topic_name, std::string frame_id, ros::NodeHandle& nh):
    topic_name(topic_name),
    nh(&nh),
    pc_pub(nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1))
{
    initPc(frame_id);
}
void Sensor2PcManager::initPc(std::string frame_id)
{
    pc.header.frame_id = frame_id;
    pc.height = 1;
    pc.width = 0;
    pc.is_bigendian = false;
    pc.is_dense = true;
    // passes cloud by reference
}

void Sensor2PcManager::addPoint(float x, float y, float z)
{

    // append one point to the point cloud
    sensor_msgs::PointCloud2Modifier modifier(pc);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pc.width + 1);
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");


    
    if (iter_x != iter_x.end() &&
        iter_y != iter_y.end() &&
        iter_z != iter_z.end()) {
        ROS_ERROR("Point cloud iterators are invalid");
        return;
    }
    iter_x[pc.width] = x;
    iter_y[pc.width] = y;
    iter_z[pc.width] = z;

    if (pc.fields.empty()) {
        ROS_ERROR("Point cloud message has no fields");
        return;
    }

    // Check if point cloud format is "xyz"
    if (pc.fields.size() != 3 ||
        pc.fields[0].name != "x" ||
        pc.fields[1].name != "y" ||
        pc.fields[2].name != "z") {
        ROS_ERROR("Point cloud format is not 'xyz'");
        return;
    }

    
}

void Sensor2PcManager::publishPc()
{
    pc_pub.publish(pc);
}

void Sensor2PcManager::publish(float x, float y, float z)
{
    ROS_INFO("Adding point");
    addPoint(x, y, z);
    ROS_INFO("Publishing point cloud");
    publishPc();
}

Sensor2PcManager* Sensor2PcManager::getInstance( ros::NodeHandle& nh, std::string frame_id, std::string topic_name)
{ 
    if (!is_initialized)
    {
        singleton = new Sensor2PcManager(topic_name, frame_id, nh);
        is_initialized = true;
    }
    return singleton;
}

Sensor2PcManager::~Sensor2PcManager()
{
    is_initialized = false;
}
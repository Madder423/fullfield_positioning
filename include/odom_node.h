#ifndef __ODOM_NODE_H
#define __ODOM_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/vector3.hpp"
#include "bupt_can/bupt_can.h"
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include <time.h>


// 全场定位数据
// typedef struct
// {
//     float vx;//x轴速度(m/s)
//     float x;//x轴坐标(m)
//     float vy;//y轴速度(m/x)
//     float y;//y轴坐标(m)
//     float vyaw;//偏航角速度(dps)
//     float yaw;//偏航角(degree)
// } fullFieldPositioning_data;
typedef struct
{
    geometry_msgs::msg::Vector3 footprint_pose;
    geometry_msgs::msg::Vector3 footprint_twist;
    double yaw;
    double vyaw;
} fullFieldPositioning_data;

// 四元数
typedef struct
{
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

class OdometerPublisher : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    OdometerPublisher(std::string name,std::string agreement, std::string odom_frame = "odom", std::string base_frame = "base_link", std::string FFP_frame = "footprint");
    //获取X_id数据
    void getXdata(const std::shared_ptr<can_frame> &frame);
    //获取Y_id轴数据
    void getYdata(const std::shared_ptr<can_frame> &frame);
    //获取Angle_id数据
    void getYawData(const std::shared_ptr<can_frame> &frame);
    //发布里程计消息
    void publish_msg();
    //发布坐标变换信息
    void publish_tf();
private:
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    //话题发布周期  默认15毫秒
    unsigned int topic_posting_cycle;
    //话题发布周期参数（默认15）
    unsigned int timer_callback_period;
    // 声明话题发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_node;
    // 发布的里程计消息
    nav_msgs::msg::Odometry _odom;
    //从全长定位获取的速度
    fullFieldPositioning_data _fullFieldPositioning_data;
    //四元数
    quaternion_t _quaternion;
    //odom坐标系名称，默认odom
    std::string odom_frame;
    //base坐标系名称，默认base_link
    std::string base_frame;
    //全场定位坐标系名称，默认footprint
    std::string fullFieldPositioning_frame;
    //全场定位协议,有TR,ER,目前主推ER
    std::string agreement;

    //欧拉角生成四元数
    void Euler2Quaternion(float yaw, float roll=0, float pitch=0);
    //根据受到数据生成里程计消息
    void CreateOdomMsg();
};

#endif //__ODOM_NODE_H
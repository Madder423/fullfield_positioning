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

typedef struct
{
    geometry_msgs::msg::Vector3 footprint_pose;
    geometry_msgs::msg::Vector3 footprint_twist;
    double yaw;
    double vyaw;
} fullFieldPositioning_data;

class OdometerPublisher : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    OdometerPublisher(std::string name,const std::shared_ptr<Can> can, std::string odom_frame = "odom", std::string base_frame = "base_link", std::string FFP_frame = "footprint");
    //发布里程计消息
    void publish_msg();
    //发布坐标变换信息
    void publish_tf();
private:
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    //can智能指针
    std::shared_ptr<Can> _can;
    // 声明话题发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_node;
    // tf发布者
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    // 发布的里程计消息
    nav_msgs::msg::Odometry _odom;
    // 原始全场定位数据
    fullFieldPositioning_data _data;
    //odom坐标系名称，默认odom
    std::string odom_frame;
    //base坐标系名称，默认base_link
    std::string base_frame;
    //全场定位坐标系名称，默认footprint
    std::string fullFieldPositioning_frame;
    //欧拉角生成四元数
    void Euler2Quaternion(float yaw, float roll=0, float pitch=0);
    //根据受到数据生成里程计消息
    void CreateOdomMsg();
    // can回调
    void getXData(const std::shared_ptr<can_frame> &frame){
    _data.footprint_twist.x=(*(float*)frame->data);
    _data.footprint_pose.x=(*(float*)(frame->data+4));
}
    void getYData(const std::shared_ptr<can_frame> &frame){
    _data.footprint_twist.y=(*(float*)frame->data);
    _data.footprint_pose.y=(*(float*)(frame->data+4));
}
    void getYawData(const std::shared_ptr<can_frame> &frame){
    _data.vyaw=(*(float*)frame->data) * 0.01745329f  ;//系数用于角度转弧度
    _data.yaw=(*(float*)(frame->data+4)) * 0.01745329f ;
}
};

#endif //__ODOM_NODE_H

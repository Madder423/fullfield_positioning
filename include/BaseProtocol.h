#pragma once
#include<iostream>
#include "bupt_can/bupt_can.h"
#include "geometry_msgs/msg/vector3.hpp"

typedef struct
{
    geometry_msgs::msg::Vector3 footprint_pose;
    geometry_msgs::msg::Vector3 footprint_twist;
    double yaw;
    double vyaw;
} fullFieldPositioning_data;

class BaseProtocol{
protected:
    std::string _protocol_type;  //协议类型
    std::shared_ptr<Can> _can; //can智能指针
    std::shared_ptr<fullFieldPositioning_data> _data; //全场定位数据
    BaseProtocol(const std::string protocol_type, const std::shared_ptr<Can> can): _protocol_type(protocol_type), _can(can){ 
        _data = std::make_shared<fullFieldPositioning_data>();
    };
    ~BaseProtocol(){};
public:
    virtual std::shared_ptr<fullFieldPositioning_data> get_data(){return _data;}
};
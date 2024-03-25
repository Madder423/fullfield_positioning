#include "ER_protocol.h"
void ER_protocol::getPoseData(const std::shared_ptr<can_frame> &frame){
    _data->footprint_pose.x=(*(float*)frame->data);
    _data->footprint_pose.y=(*(float*)(frame->data+4));
}
void ER_protocol::getVelData(const std::shared_ptr<can_frame> &frame){
    _data->footprint_twist.x=(*(float*)frame->data);
    _data->footprint_twist.y=(*(float*)(frame->data+4));
}
void ER_protocol::getYawData(const std::shared_ptr<can_frame> &frame){
    _data->vyaw=(*(float*)frame->data);
    _data->yaw=(*(float*)(frame->data+4));
}
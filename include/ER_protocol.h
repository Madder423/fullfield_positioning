#include "BaseProtocol.h"
class ER_protocol:public BaseProtocol{
private:
    void getPoseData(const std::shared_ptr<can_frame> &frame);
    void getVelData(const std::shared_ptr<can_frame> &frame);
    void getYawData(const std::shared_ptr<can_frame> &frame);
public:
    ER_protocol(const std::shared_ptr<Can> can):BaseProtocol("ER",can){
        std::cout<<"using protocol:"<<_protocol_type<<std::endl;
        can->register_msg(0x0CE, Can::CAN_ID_STD, std::bind(&ER_protocol::getYawData, this,std::placeholders::_1));
        can->register_msg(0x0CC, Can::CAN_ID_STD, std::bind(&ER_protocol::getPoseData, this,std::placeholders::_1));
        can->register_msg(0x0CD, Can::CAN_ID_STD, std::bind(&ER_protocol::getVelData, this,std::placeholders::_1));
    };
};
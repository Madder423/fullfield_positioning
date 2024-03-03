#include "bupt_can/bupt_can.h"
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
//#include "rclcpp/rclcpp.hpp"
#include "odom_node.h"


// void signalHandler( int signum )
// {
//     std::cout << "Interrupt signal (" << signum << ") received.\n";

//     // 清理并关闭
//     // 终止程序  
  
//    rclcpp::shutdown();
//    //exit(signum);  

// }



int main(int argc,char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个的节点*/
    //OdometerPublisher node("fullFieldPositioning_node");
    auto node = std::make_shared<OdometerPublisher>("fullFieldPositioning_node","ER");
    //OdometerPublisher node("fullFieldPositioning_node");
    Can can("can0");
    
    can.register_msg(0x0CE, Can::CAN_ID_STD, std::bind(&OdometerPublisher::getYawData, node ,std::placeholders::_1));
    can.register_msg(0x0CC, Can::CAN_ID_STD, std::bind(&OdometerPublisher::getXdata, node,std::placeholders::_1));
    can.register_msg(0x0CD, Can::CAN_ID_STD, std::bind(&OdometerPublisher::getYdata, node ,std::placeholders::_1));
    // can.register_msg(0x0CE, Can::CAN_ID_STD, std::bind(print_the_msg,std::placeholders::_1));
    // can.register_msg(0x0CD, Can::CAN_ID_STD, std::bind(print_the_msg,std::placeholders::_1));
    // can.register_msg(0x0CC, Can::CAN_ID_STD, std::bind(print_the_msg,std::placeholders::_1));
    // 注册CAN消息回调函数
    // 这个函数会把对应的CAN消息的ID和回调函数绑定起来
    // 并且会把对应的ID添加到过滤器中
    can.can_start();
    // 这一函数会设置过滤器并且启动收发线程
    // 一个推荐的做法是在注册完所有消息后调用一次这个函数
    // 在启动收发线程之后你不应该尝试修改过滤器
    // 这一行为未经测试，可能会导致未知的错误

    // std::thread send_thread([&can](){
    //     while (true)
    //     {
    //         std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //         std::array<uint8_t,8> data = {0x1F,0xFF,0x00,0x00,0x00,0x00,0x00};
    //         can.send_can(0x12345678,Can::CAN_ID_EXT,8,data);
    //         can.send_can(0x205,Can::CAN_ID_STD,8,data);
    //         // 请注意，不要无延时全速发消息
    //         // 这样会导致CAN总线拥塞
    //         // 这样的场景很罕见，但是还是要注意
    //         // 用专业CAN卡测试，发送频率在1khz时已经几乎达到极限
    //     }
    // });
    // send_thread.detach();

    //signal(SIGINT, signalHandler);
     /* 运行节点，并检测退出信号 Ctrl+C*/
    rclcpp::spin(node);
        /* 停止运行 */
    rclcpp::shutdown();
    // while (true)
    // {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // }   
    return 0;
}
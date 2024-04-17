#include "bupt_can/bupt_can.h"
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <functional>
#include "odom_node.h"

int main(int argc,char **argv)
{
    rclcpp::init(argc, argv);
    //实例化can
    std::shared_ptr<Can> can = std::make_shared<Can>("can0");
    /*产生一个的节点*/
    //OdometerPublisher node("fullFieldPositioning_node");
    auto node = std::make_shared<OdometerPublisher>("fullFieldPositioning_node", can);
    can->can_start();

    rclcpp::spin(node);
        /* 停止运行 */
    rclcpp::shutdown();

    return 0;
}
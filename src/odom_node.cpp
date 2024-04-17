#include "odom_node.h"

OdometerPublisher::OdometerPublisher(std::string name,const std::shared_ptr<Can> can, std::string odom_frame, std::string base_frame, std::string FFP_frame) : 
Node(name), _can(can), odom_frame(odom_frame), base_frame(base_frame), fullFieldPositioning_frame(FFP_frame)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        // 创建发布者
        odom_node = this->create_publisher<nav_msgs::msg::Odometry>("odom",10);
        // 创建一个tf2_ros::TransformBroadcaster用于广播坐标变换
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        // 创建定时器，15ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(15), std::bind(&OdometerPublisher::publish_msg, this));
        //声明参数
        //this->declare_parameter<std::int64_t>("topic_posting_cycle",topic_posting_cycle);
        _can->register_msg(0x0CC, Can::CAN_ID_STD, std::bind(&OdometerPublisher::getXData, this,std::placeholders::_1));
        _can->register_msg(0x0CD, Can::CAN_ID_STD, std::bind(&OdometerPublisher::getYData, this,std::placeholders::_1));
        _can->register_msg(0x0CE, Can::CAN_ID_STD, std::bind(&OdometerPublisher::getYawData, this,std::placeholders::_1));
    }

void OdometerPublisher::CreateOdomMsg()
{
    //消息头
    _odom.header.stamp.sec = this->now().seconds();
    _odom.header.stamp.nanosec = (this->now().nanoseconds()%1000000000);
    //std::cout<<_odom.header.stamp.sec<<std::endl;
    //std::cout<<_odom.header.stamp.nanosec<<std::endl;
                                                                        
    _odom.header.frame_id = odom_frame;
    _odom.child_frame_id = base_frame;

    // tf监听
    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::msg::TransformStamped transformStamped;
    tfBuffer.reset(new tf2_ros::Buffer(this->get_clock()));
    tf_listener.reset(new tf2_ros::TransformListener(*tfBuffer));
    // 等待并获取目标坐标系到源坐标系的变换
    try
    {
        transformStamped = tfBuffer->lookupTransform(base_frame, fullFieldPositioning_frame, tf2::TimePointZero,tf2::durationFromSec(1.0));
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("TF转换错误"), "Could not transform point.");
    }
    // 使用 transformStamped 的变换信息对位姿进行转换
    // 姿态
    tf2::doTransform(_data.footprint_pose, _data.footprint_pose, transformStamped);
    _odom.pose.pose.position.x = _data.footprint_pose.x;
    _odom.pose.pose.position.y = _data.footprint_pose.y;
    Euler2Quaternion(_data.yaw);
    // 速度
    tf2::doTransform(_data.footprint_twist, _data.footprint_twist, transformStamped);
    _odom.twist.twist.angular.z = _data.vyaw;
    _odom.twist.twist.linear.x = _data.footprint_twist.x;
    _odom.twist.twist.linear.y = _data.footprint_twist.y;

}
void OdometerPublisher::Euler2Quaternion(float yaw, float roll, float Pitch)

{
    // 传入机器人的欧拉角 roll、Pitch 和 yaw。
    // 计算欧拉角的 sin 和 cos 值，分别保存在 cr、sr、cy、sy、cp、sp 六个变量中   
    // https://blog.csdn.net/xiaoma_bk/article/details/79082629
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(Pitch * 0.5);
    double sp = sin(Pitch * 0.5);
    // 计算出四元数的四个分量 q.w、q.x、q.y、q.z
    _odom.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr;
    _odom.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr;
    _odom.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr;
    _odom.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr;
    //std::cout<<_quaternion.w<<std::endl;
}

void OdometerPublisher::publish_msg()
{
    CreateOdomMsg();
    // 日志打印
    //RCLCPP_INFO(this->get_logger(),"\t\nx=%f\t\nvx=%f\t\ny=%f\t\nvy=%f\t\nyaw=%f\t\nvyaw:%f\t\n",_odom.pose.pose.position.x,_odom.twist.twist.linear.x,_odom.pose.pose.position.y,_odom.twist.twist.linear.y,_fullFieldPositioning_data->yaw,_odom.twist.twist.angular.z);
    // 发布TF
    publish_tf();
    // 发布消息
    odom_node->publish(_odom);
}

void OdometerPublisher::publish_tf()
{
    geometry_msgs::msg::TransformStamped transform;
    double seconds = this->now().seconds();
    transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
    transform.header.frame_id = odom_frame;
    transform.child_frame_id = base_frame;
    //test_code
    std::cout<<"frame_id:"<<odom_frame<<"\tchild_frame_id:"<<base_frame<<std::endl;
    transform.transform.translation.x = _odom.pose.pose.position.x;
    transform.transform.translation.y = _odom.pose.pose.position.y;
    transform.transform.translation.z = _odom.pose.pose.position.z;
    transform.transform.rotation.x = _odom.pose.pose.orientation.x;
    transform.transform.rotation.y = _odom.pose.pose.orientation.y;
    transform.transform.rotation.z = _odom.pose.pose.orientation.z;
    transform.transform.rotation.w = _odom.pose.pose.orientation.w;

    // 广播坐标变换信息
    tf_broadcaster_->sendTransform(transform);
}


#pragma once
#include "ros2socketcan.h" // 引入 ros2socketcan 头文件
#include "can_msgs/msg/frame.hpp" // 引入 CAN 数据类型
#include "rclcpp/rclcpp.hpp"


class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node")
    {
        // 定时器触发，定期发送 CAN 消息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000), // 每 1 秒触发
            std::bind(&MyNode::send_can_data, this));
        // send_can_data();
        // timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(1000), // 每 1 秒触发
        //     std::bind(&MyNode::send_can_data, this));
        // // 启动定时器，只执行一次
        // timer_->reset();
    }

private:

    std::string topic_receive = "CAN/can1/receive";
    std::string topic_transmit = "CAN/can1/transmit";

    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscribe_can_msg = this->create_subscription<can_msgs::msg::Frame>(topic_receive, 100, std::bind(&MyNode::CanSubscribe, this, std::placeholders::_1));

    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publish_can_msg = this->create_publisher<can_msgs::msg::Frame>(topic_transmit, 10);

    void CanSubscribe(const can_msgs::msg::Frame::SharedPtr msg)
    {
        std::stringstream out;
        out << std::string("S | ") << std::to_string(msg->id) << std::string("| ");
        for (int j = 0; j < (int)msg->dlc; j++)
        {
            out << std::to_string(msg->data[j]) << std::string(" ");
        }
        out << std::endl;
        RCLCPP_INFO(this->get_logger(), out.str().c_str());
    }

    void send_can_data()
    {
        // 创建并设置 CAN 消息
        can_msgs::msg::Frame can_msg;
        can_msg.id = 0x036;               // 设置 CAN ID
        can_msg.dlc = 8;                  // 数据长度 (最大8字节)
        can_msg.is_extended = false;      // 标准帧
        can_msg.is_rtr = false;           // 非远程请求
        can_msg.is_error = false;         // 无错误标志
        can_msg.data = {0x01, 0x1C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}; // 数据内容
        publish_can_msg->publish(can_msg);

        // rclcpp::sleep_for(std::chrono::milliseconds(10)); 

        can_msg.id = 0x038;     
        can_msg.data = {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02}; 
        publish_can_msg->publish(can_msg);

        rclcpp::sleep_for(std::chrono::milliseconds(1000));   //使能电机之后要停1s才能使用

        can_msg.id = 0x101;     
        can_msg.data = {0x00, 0x00, 0xA1, 0x70, 0x00, 0x60, 0x08, 0x00}; 
        
        publish_can_msg->publish(can_msg);

        can_msg.data = {0x00, 0x00, 0x91, 0x70, 0x00, 0x60, 0x08, 0x00};        
        publish_can_msg->publish(can_msg);
        // 调用 ros2socketcan 的发送方法
    
        // RCLCPP_INFO(this->get_logger(), "Sent CAN data with ID: 0x%x", can_msg.id);
    }

    rclcpp::TimerBase::SharedPtr timer_;     // 定时器
};
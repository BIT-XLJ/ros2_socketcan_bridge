// Original Author: Philipp Wuestenberg <philipp.wuestenberg@tu-berlin.de>
// Maintainer: Jonathan Blixti <blixt013@umn.edu>
// Last Updated: November 2023

// Import header file
#include "ros2socketcan.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

ros2socketcan::ros2socketcan(const std::string& can_device_name, const std::string& node_name)
    : Node(node_name),  // 使用传入的自定义节点名称
      stream(ios), 
      signals(ios, SIGINT, SIGTERM)
{
    // 使用传入的can_device_name参数
    this->declare_parameter("CAN_INTERFACE", can_device_name);
    std::string can_socket = this->get_parameter("CAN_INTERFACE").as_string();

    RCLCPP_INFO(this->get_logger(), "CAN_INTERFACE: %s", can_socket.c_str());
    topicname_receive << "CAN/" << can_socket << "/"
                      << "receive"; //从can总线接收到信息
    topicname_transmit << "CAN/" << can_socket << "/"
                       << "transmit"; //向can总线发送信息

    publisher_ = this->create_publisher<can_msgs::msg::Frame>(topicname_receive.str(), 10);
    subscription_ = this->create_subscription<can_msgs::msg::Frame>(topicname_transmit.str(), 100, std::bind(&ros2socketcan::CanPublisher, this, _1));

    strcpy(ifr.ifr_name, can_socket.c_str());
    ioctl(natsock, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(natsock, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Error in socket bind");
    }

    stream.assign(natsock);

    RCLCPP_INFO(this->get_logger(), (std::string("ROS 2 node ") + node_name + std::string(" to CAN-Bus topic:") + subscription_->get_topic_name()).c_str());
    RCLCPP_INFO(this->get_logger(), (std::string("CAN-Bus to ROS 2 node ") + node_name + std::string(" topic:") + publisher_->get_topic_name()).c_str());

    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)), std::bind(&ros2socketcan::CanListener, this, std::ref(rec_frame), std::ref(stream)));
    signals.async_wait(std::bind(&ros2socketcan::stop, this));

    std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
    std::thread bt(std::bind(run, &ios));
    RCLCPP_DEBUG(this->get_logger(),"Thread started");
    bt.detach();
}

void ros2socketcan::stop()
{
    RCLCPP_INFO(this->get_logger(), "End of Listener Thread. Please press strg+c again to stop the whole program.\n");
    ios.stop();
    signals.clear();
}

ros2socketcan::~ros2socketcan()
{
    RCLCPP_INFO(this->get_logger(), "End of Publisher Thread. \n");
}

void ros2socketcan::CanSend(const can_msgs::msg::Frame msg)
{
    struct can_frame frame1;

    frame1.can_id = msg.id;

    if (msg.is_extended == true)
    {
        frame1.can_id = frame1.can_id + CAN_EFF_FLAG;
    }

    if (msg.is_error == true)
    {
        frame1.can_id = frame1.can_id + CAN_ERR_FLAG;
    }

    if (msg.is_rtr == true)
    {
        frame1.can_id = frame1.can_id + CAN_RTR_FLAG;
    }

    frame1.can_dlc = msg.dlc;

    for (int i = 0; i < (int)frame1.can_dlc; i++)
    {
        frame1.data[i] = msg.data[i];
    }

    stream.async_write_some(boost::asio::buffer(&frame1, sizeof(frame1)), std::bind(&ros2socketcan::CanSendConfirm, this));
}

// Publish messages to the CAN bus
void ros2socketcan::CanPublisher(const can_msgs::msg::Frame::SharedPtr msg)
{
    can_msgs::msg::Frame msg1;
    msg1.id = msg->id;
    msg1.dlc = msg->dlc;
    msg1.is_extended = msg->is_extended;
    msg1.is_rtr = msg->is_rtr;
    msg1.is_error = msg->is_error;
    msg1.data = msg->data;

    CanSend(msg1);
}
void ros2socketcan::CanSendConfirm(void)
{
    
}
// Listen for CAN messages
void ros2socketcan::CanListener(struct can_frame &rec_frame, boost::asio::posix::basic_stream_descriptor<> &stream)
{
    can_msgs::msg::Frame frame;

    frame.id = rec_frame.can_id;
    frame.dlc = int(rec_frame.can_dlc);

    for (int i = 0; i < rec_frame.can_dlc; i++)
    {
        frame.data[i] = rec_frame.data[i];
    }

    publisher_->publish(frame);

    stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)), std::bind(&ros2socketcan::CanListener, this, std::ref(rec_frame), std::ref(stream)));
}

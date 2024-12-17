#pragma once
#include "ros2socketcan.h" // 引入 ros2socketcan 头文件
#include "can_msgs/msg/frame.hpp" // 引入 CAN 数据类型
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <cstdint>
#include "std_msgs/msg/int32.hpp"
class CanNode : public rclcpp::Node
{
public:
    CanNode(const std::string& node_name) : Node(node_name)
    {
        can_node_name = node_name;
        topic_receive = std::string("CAN/") + can_node_name + std::string("/receive");
        topic_transmit = std::string("CAN/") + can_node_name + std::string("/transmit");
        subscribe_can_msg = this->create_subscription<can_msgs::msg::Frame>(topic_receive, 100, std::bind(&CanNode::CanSubscribe, this, std::placeholders::_1));
        publish_can_msg = this->create_publisher<can_msgs::msg::Frame>(topic_transmit, 10);
        motor_init_sub = this->create_subscription<std_msgs::msg::Int32>(
            "motor_init", 10,  // 话题名，队列大小
            std::bind(&CanNode::topic_callback, this, std::placeholders::_1));
    }

    void init_motor(int ID, bool reset_zero_pos)
    {
        can_msgs::msg::Frame can_msg;
        can_msg.dlc = 8;                  // 数据长度 (最大8字节)
        can_msg.is_extended = false;      // 标准帧
        can_msg.is_rtr = false;           // 非远程请求
        can_msg.is_error = false;         // 无错误标志

        if(ID==1)
        {
            can_msg.id = 0x036;     
            can_msg.data = {0x01, 0x1C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};  //ID号为1，设置mit模式
            publish_can_msg->publish(can_msg);
            can_msg.id = 0x034;     
            can_msg.data = {0x01, 0x01, 0x01, 0x0D, 0x01, 0x03, 0x05, 0x00};  //反馈位置 速度 Q轴电流（和输出力矩成正比关系） 故障信息 设置反馈信息不会保存，需要每次上电都给出指令
            publish_can_msg->publish(can_msg);                                //主动上报反馈的ID号为C0+电机ID
        }
        else if(ID==2)
        {
            can_msg.id = 0x036;     
            can_msg.data = {0x02, 0x1C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};  //ID号为2，设置mit模式
            publish_can_msg->publish(can_msg);
            can_msg.id = 0x034;     
            can_msg.data = {0x02, 0x01, 0x01, 0x0D, 0x01, 0x03, 0x05, 0x00};  //反馈位置 速度 Q轴电流（和输出力矩成正比关系） 故障信息 设置反馈信息不会保存，需要每次上电都给出指令
            publish_can_msg->publish(can_msg);                                //主动上报反馈的ID号为C0+电机ID            
        }
        else if(ID==3)
        {
            can_msg.id = 0x036;     
            can_msg.data = {0x03, 0x1C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};  //ID号为3，设置mit模式
            publish_can_msg->publish(can_msg);
            can_msg.id = 0x034;     
            can_msg.data = {0x03, 0x01, 0x01, 0x0D, 0x01, 0x03, 0x05, 0x00};  //反馈位置 速度 Q轴电流（和输出力矩成正比关系） 故障信息 设置反馈信息不会保存，需要每次上电都给出指令
            publish_can_msg->publish(can_msg);                                //主动上报反馈的ID号为C0+电机ID
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Can ID should not be greater than 3, init motor fail!");
            return;
        }

        if(reset_zero_pos == true)
        {
            can_msg.id = 0x039;   
            can_msg.data = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};      
            publish_can_msg->publish(can_msg);                                 // 这个命令是设置绝对零位，设置完绝对零位之后最好是断电重启
        }

    }

    void enable_all_motors()
    {
        can_msgs::msg::Frame can_msg;
        can_msg.dlc = 8;                  // 数据长度 (最大8字节)
        can_msg.is_extended = false;      // 标准帧
        can_msg.is_rtr = false;           // 非远程请求
        can_msg.is_error = false;         // 无错误标志
        can_msg.id = 0x038;     
        can_msg.data = {0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02};      //使能
        publish_can_msg->publish(can_msg);
        rclcpp::sleep_for(std::chrono::seconds(3));
    }

    void mit_mode_send(int ID, double Kp, double Kd, double p_des, double v_des, double t_ff) //输入position单位是rad，范围是[-pi,pi];输入velocity单位是rad/s；输入力矩需要转换为电流
    {
        can_msgs::msg::Frame can_msg;
        can_msg.dlc = 8;                  // 数据长度 (最大8字节)
        can_msg.is_extended = false;      // 标准帧
        can_msg.is_rtr = false;           // 非远程请求
        can_msg.is_error = false;         // 无错误标志

        if(ID == 1)
        {
           can_msg.id = 0x101;    
        }
        else if(ID == 2)
        {
           can_msg.id = 0x102;   
        }
        else if(ID == 3)
        {
           can_msg.id = 0x103;   
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Can ID should not be greater than 3, mit control fail!");
            return;            
        }
        
        double position  = -p_des; //本末的电机和右手螺旋定则是反过来的
        double velocity  = -v_des;
        double torque_ff = -(t_ff * 1.414 / 1.2);  //t_ff是力矩，torque_ff是电流
        
        // 对position进行限幅，范围为[-pi, pi]
        // position = std::fmax(std::fmin(position, M_PI), -M_PI);
    
        // 对velocity进行限幅，范围为[-20, 20] rad/s
        velocity = std::fmax(std::fmin(velocity, 20.0), -20.0);
        
        // 对torque_ff进行限幅，范围为[-50, 50] A 
        torque_ff = std::fmax(std::fmin(torque_ff, 50.0), -50.0);

        // 对Kp进行限幅，范围为[0, 50]
        Kp = std::fmax(std::fmin(Kp, 50.0), 0.0);

        // 对Kd进行限幅，范围为[0, 5]
        Kd = std::fmax(std::fmin(Kd, 5.0), 0.0);

        uint16_t p_des_CAN = static_cast<uint16_t>(std::round((position + 100.0 * M_PI) / (200.0 * M_PI) * 65536));
        uint16_t v_des_CAN = static_cast<uint16_t>(std::round((velocity + 20.0) * 4096.0 / 40.0));
        uint16_t Kp_CAN = static_cast<uint16_t>(std::round(Kp * 4096.0 / 500.0));
        uint16_t Kd_CAN = static_cast<uint16_t>(std::round(Kd * 4096.0 / 5.0));
        uint16_t t_ff_CAN = static_cast<uint16_t>(std::round((torque_ff + 50.0) * 4096.0 / 100.0));
        // 拆分并填充到can_msg.data中
        can_msg.data[0] = static_cast<uint8_t>((p_des_CAN >> 8) & 0xFF);  // p_des[15:8]
        can_msg.data[1] = static_cast<uint8_t>(p_des_CAN & 0xFF);         // p_des[7:0]
        can_msg.data[2] = static_cast<uint8_t>((v_des_CAN >> 4) & 0xFF);   // v_des[11:4]
        can_msg.data[3] = static_cast<uint8_t>(((v_des_CAN & 0xF) << 4) | ((Kp_CAN >> 8) & 0xF));  // v_des[3:0] | Kp[11:8]
        can_msg.data[4] = static_cast<uint8_t>(Kp_CAN & 0xFF);             // Kp[7:0]
        can_msg.data[5] = static_cast<uint8_t>((Kd_CAN >> 4) & 0xFF);      // Kd[11:4]
        can_msg.data[6] = static_cast<uint8_t>(((Kd_CAN & 0xF) << 4) | ((t_ff_CAN >> 8) & 0xF)); // Kd[3:0] | t_ff[11:8]
        can_msg.data[7] = static_cast<uint8_t>(t_ff_CAN & 0xFF);           // t_ff[7:0]

        //本末的电机和右手螺旋定则是反过来的，也就是我们的控制指令需要全部给一个相反数，并且，mit模式设置的速度是rad/s，但是电机反馈的速度是RMP，并且mit模式设置的位置还是多圈位置
        publish_can_msg->publish(can_msg);
    }

private:
    std::string can_node_name;
    std::string topic_receive;
    std::string topic_transmit;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscribe_can_msg;
    rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publish_can_msg;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motor_init_sub; 

    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        this->init_motor(3,false);
        this->enable_all_motors();
        this->mit_mode_send(3, 0.0, 0.0, 100 * M_PI, 2.00, 2.5);  // 延迟后执行发送
    }

    void CanSubscribe(const can_msgs::msg::Frame::SharedPtr msg) //从can总线接收到信息，在这里处理
    {
        if(msg->id == 0xC3)
        {
            int16_t position_raw = (static_cast<int16_t>(msg->data[0]) << 8) | msg->data[1];
            if (position_raw > 32767) {
                position_raw -= 65536;  // 转换为有符号整数
            }
            double position = 0 - position_raw * M_PI / 32768;  // 转换为弧度。取相反数，因为本末电机很奇怪

            // 2. 解析电机转速
            int16_t speed_raw = (static_cast<int16_t>(msg->data[2]) << 8) | msg->data[3];
            double speed_rpm = speed_raw / 10.0;  // 转速的实际值
            double speed_rad_per_s = 0 - speed_rpm * 2 * M_PI / 60.0;  // 转换为rad/s

            // 3. 解析电机Q轴电流
            int16_t current_raw = (static_cast<int16_t>(msg->data[4]) << 8) | msg->data[5];
            if (current_raw > 32767) {
                current_raw -= 65536;  // 转换为有符号整数
            }
            double current = 0 - current_raw / 100.0;  // 转换为电流值（单位：A）

            // 输出结果（可以根据需求处理）
            std::cout << "电机位置: " << position << " rad" << std::endl;
            std::cout << "电机转速: " << speed_rad_per_s << " rad/s" << std::endl;
            std::cout << "电机Q轴电流: " << current << " A" << std::endl;
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;     // 定时器
};
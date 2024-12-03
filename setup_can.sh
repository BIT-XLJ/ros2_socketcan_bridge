#!/bin/bash

# 1. 配置串口设备为 CAN 设备
echo "Setting up CAN interface using slcand..."
sudo slcand -o -c -s8 /dev/ttyACM1 can1

# 2. 启动 can0 接口
echo "Bringing up the CAN interface (can1)..."
sudo ifconfig can1 up

# 3. 设置 CAN 接口的发送队列长度
echo "Setting txqueuelen for can1 to 10000..."
sudo ifconfig can1 txqueuelen 10000

# 打印接口状态
echo "CAN interface can1 is up and running with txqueuelen set to 10000."
ifconfig can1

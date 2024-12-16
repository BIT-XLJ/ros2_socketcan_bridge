#!/bin/bash

# 定义配置CAN接口的函数
configure_can_interface() {
    local device=$1
    local can_interface=$2

    echo "Setting up CAN interface using slcand on $device..."
    sudo slcand -o -c -s8 $device $can_interface

    echo "Bringing up the CAN interface ($can_interface)..."
    sudo ifconfig $can_interface up

    echo "Setting txqueuelen for $can_interface to 100000..."
    sudo ifconfig $can_interface txqueuelen 100000

    echo "CAN interface $can_interface is up and running with txqueuelen set to 100000."
    ifconfig $can_interface
}

# 检查 /dev/ttyACM0 是否存在
if [ -e /dev/ttyACM0 ]; then
    echo "/dev/ttyACM0 found. Proceeding with CAN setup..."
    configure_can_interface /dev/ttyACM0 can0
else
    echo "Error: /dev/ttyACM0 not found."
fi

# 检查 /dev/ttyACM1 是否存在
if [ -e /dev/ttyACM1 ]; then
    echo "/dev/ttyACM1 found. Proceeding with CAN setup..."
    configure_can_interface /dev/ttyACM1 can1
else
    echo "Error: /dev/ttyACM1 not found."
fi

# 检查 /dev/ttyACM2 是否存在
if [ -e /dev/ttyACM2 ]; then
    echo "/dev/ttyACM2 found. Proceeding with CAN setup..."
    configure_can_interface /dev/ttyACM2 can2
else
    echo "Error: /dev/ttyACM2 not found."
fi

# 检查 /dev/ttyACM3 是否存在
if [ -e /dev/ttyACM3 ]; then
    echo "/dev/ttyACM3 found. Proceeding with CAN setup..."
    configure_can_interface /dev/ttyACM3 can3
else
    echo "Error: /dev/ttyACM3 not found."
fi


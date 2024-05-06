#ifndef MOTOR_H__
#define MOTOR_H__


#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
//#include <ebot_base/FlagStatus.h>

#define FRAME_HEADER_SOF    (0xA5)
#define FRAME_HEADER_LEN    (3)
#define FRAME_CMD_LEN       (1)
#define FRAME_TAIL_LEN      (2)

enum class UnpackStep {
    HEADER_SOF,
    HEADER_LEN,
    HEADER_CRC8,
    TAIL_CRC16,
};

serial::Serial ss;  // 声明串口对象
// 参数
std::string port_;
int baudrate_;
int mode_;


#endif  
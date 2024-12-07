#ifndef ROS_32_H
#define ROS_32_H

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include<serial/serial.h>

extern void serialInit(boost::asio::serial_port& sp);//初始化串口
void serialOpen(boost::asio::serial_port& sp);//打开串口
extern void serialClose(boost::asio::serial_port& sp); //关闭串口
extern void sendcoord(float x, float y, float z, boost::asio::serial_port& sp);//发送机器人坐标信息
extern void sendpose(float roll, float pitch, float yaw, boost::asio::serial_port& sp);//发送机器人姿态信息
extern bool serialIsOpen(boost::asio::serial_port& sp);//判断串口是否打开
unsigned char getCrc8(unsigned char *data, unsigned short len); //计算CRC8校验码

#endif // ROS_32_H
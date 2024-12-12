#ifndef ROS_32_H
#define ROS_32_H

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <boost/asio/serial_port.hpp>


using namespace std;
extern unsigned char msg_place[];
extern void serialInit();//初始化串口
extern void sendTest();//发送测试数据
extern void send24byte(unsigned char* msg);//发送24字节数据
extern void receiveTest(unsigned char* msg);//接收测试数据
extern int receive24byte(unsigned char* msg);//接收24字节数据
unsigned char getCrc8(unsigned char *data, unsigned short len); //计算CRC8校验码
void depart_place(const std_msgs::String::ConstPtr& msg);//拆分ros消息

#endif // ROS_32_H
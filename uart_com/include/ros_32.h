#ifndef ROS_32_H
#define ROS_32_H

#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

using namespace std;

extern void serialInit();//初始化串口
//extern void serialOpen();//打开串口
extern void serialClose(); //关闭串口
extern void sendTest();//发送测试数据
extern void send24byte(float& a, float& b, float& c, float& d, float& e, float& f,int precision1,int precision2);//发送24字节数据
extern void receiveTest();//接收测试数据
extern void receive24byte(float& a, float& b, float& c,float& d, float& e, float& f,int precision1,int precision2);//接收24字节数据
extern bool serialIsOpen();//判断串口是否打开
unsigned char getCrc8(unsigned char *data, unsigned short len); //计算CRC8校验码

#endif // ROS_32_H
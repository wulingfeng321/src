//从ros话题中读取数据，并输出到串口中
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<iostream>
#include<string>
#include<sstream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include "../include/ros_32.h"

//place_info拆分为位置和姿态并发送
void send_into_ttl(const std_msgs::String::ConstPtr& msg);

//主函数
int main(int argc, char** argv){
    //初始化节点
    ros::init(argc, argv, "uart_node");
    //创建节点句柄
    ros::NodeHandle uart;

    //初始化串口
    serialInit();//初始化串口sp

    //检测串口是否打开
    if(!serialIsOpen()){
        ROS_ERROR("sp 启动失败！");
        return -1;
    }
    else{
        ROS_INFO("sp 启动成功！");
    }


    //ros
    ros::Rate loop_rate(50); //指定循环频率50 
    while(ros::ok()){
        //订阅者创建
		//订阅place_info话题，包含机器人当前坐标，姿态信息
        int precision2 = 10000;
		ros::Subscriber uart_sub = uart.subscribe("place_info", 1000, send_into_ttl);

        //等待回调函数执行
		loop_rate.sleep();
		//等待一秒
		//ros::Duration(1.0).sleep();
	}
	//关闭串口
	serialClose();//关闭串口coord_io
	return 0;

}

//place_info拆分为位置和姿态并发送
void send_into_ttl(const std_msgs::String::ConstPtr& msg) {
    //将消息中的数据保存到字符串中
    std::string serial_msg = msg->data;
    //信息切分
    std::stringstream ss(serial_msg);
    float a, b, c, d, e, f;
    ss >> a >> b >> c >> d >> e >> f;
    //发送数据,保留4位小数和3位小数
    send24byte(a, b, c, d, e, f,10000,1000);
}
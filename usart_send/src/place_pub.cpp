#include<ros/ros.h>
#include<serial/serial.h>
#include<std_msgs/String.h>
#include<iostream>
#include<string>
#include<sstream>

//using namespace std; //声明命名空间

//函数功能：将数据经由串口发送出去
//入口参数1：[serial::Serial &ser]：     串口类名称
//入口参数2：[std::string &serial_msg]:  要通过串口发送出去的字符串
int serial_write(serial::Serial &ser, std::string &serial_msg)
{
    ser.write(serial_msg);
    return 0;
}

//函数功能：将从串口接收到的数据保存到数组中
//入口参数1：[serial::Serial &ser]：     串口类名称
//入口参数2：[std::string &serial_msg]:  从串口读取的字符串
int serial_read(serial::Serial &ser, std::string &serial_msg)
{
    serial_msg = ser.read( ser.available() );
    return 0;
}


int main(int argc, char** argv)
{
    //初始化，节点名为serial_publisher
    ros::init(argc, argv,"serial_publisher");
    //创建句柄seuNB，用于管理资源
    ros::NodeHandle seuNB;

    //用Publisher类，实例化一个发布者对象yao，发布一个名为"Serial_Topic"的话题，话题的消息类型为std_msgs::String，消息发布队列长度为10(注意话题名中间不能有空格)
    ros::Publisher yao = seuNB.advertise<std_msgs::String>("Serial_Topic",10);

    //实例化一个serial类
    serial::Serial ser;

    //初始化串口相关设置
    ser.setPort("/dev/pts/3");         //设置打开的串口名称:这里打开一个虚拟串口
    ser.setBaudrate(115200);           //设置串口的波特率
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);  //创建timeout
    ser.setTimeout(to);                //设置串口的timeout

    //打开串口
    try
    {
        ser.open();         //打开串口
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");        //打开串口失败，打印日志信息，然后结束程序
        return -1;
    }

    //判断串口是否成功打开
    if(ser.isOpen())
    { 
        ROS_INFO_STREAM("Serial Port is opened.\n");    //成功打开串口，打印日志信息
    }
    else
    {
        return -1;  //打开串口失败，打印日志信息，然后结束程序
    }


    ros::Rate loop_rate(50); //指定循环频率50  
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = ser.available();
        if(n!=0)
        {
            ROS_INFO_STREAM("Reading from serial port:\n"); //表明正在开始读取串口数据
            std_msgs::String msg2333; 	//msg2333为从串口处接收到的字符串
            msg2333.data = ser.read(ser.available()); 
            yao.publish(msg2333);  //将消息发布出去
            ROS_INFO_STREAM("Read: " << msg2333.data);  //添加日志：顺便将发布的数据打印到终端
        }
        
        loop_rate.sleep();
    }
    
    //关闭串口
	ser.close();
    return 0;
}
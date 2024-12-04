//从ros话题中读取数据，并输出到串口中
#include<ros/ros.h>
#include<serial/serial.h>
#include<std_msgs/String.h>
#include<iostream>
#include<string>
#include<sstream>

//实例化一个serial类
serial::Serial ser;

//函数功能：将数据经由串口发送出去
//入口参数1：[serial::Serial &ser]：     串口类名称
//入口参数2：[std::string &serial_msg]:  要通过串口发送出去的字符串
int serial_write(serial::Serial &ser, std::string &serial_msg);

//函数功能：将从串口接收到的数据保存到数组中
//入口参数1：[serial::Serial &ser]：     串口类名称
//入口参数2：[std::string &serial_msg]:  从串口读取的字符串
int serial_read(serial::Serial &ser, std::string &serial_msg);

//回调函数功能：将从串口接收到的数据保存到字符串中
//入口参数1：[const std_msgs::String::ConstPtr& msg]：  订阅的消息
void send_into_ttl(const std_msgs::String::ConstPtr& msg);

int main(int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv,"usart_send");
    //创建句柄
    ros::NodeHandle usart;

    

    //初始化串口相关设置
    ser.setPort("/dev/pts/6");         //设置打开的串口名称:这里打开一个虚拟串口
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
        ROS_WARN_STREAM("Waiting for data to be published on topic 'place_info'.");
    //创建订阅者，订阅名为“place_info”的消息
    ros::Subscriber usart_in = usart.subscribe("place_info", 1000, send_into_ttl);

    //测试数据发送
    std::string serial_msg = "Hello, world!";
    serial_write(ser, serial_msg);

    //等待回调函数执行
    loop_rate.sleep(); 
    //等待一秒
    ros::Duration(1.0).sleep();

    }
    
    //关闭串口
	ser.close();
    return 0;
}

void send_into_ttl(const std_msgs::String::ConstPtr& msg)
{
    //将消息中的数据保存到字符串中
    std::string serial_msg = msg->data;
    //将字符串通过串口发送出去
    serial_write(ser, serial_msg);
}

int serial_read(serial::Serial &ser, std::string &serial_msg)
{
    serial_msg = ser.read( ser.available() );
    return 0;
}

int serial_write(serial::Serial &ser, std::string &serial_msg)
{
    ser.write(serial_msg);
    return 0;
}
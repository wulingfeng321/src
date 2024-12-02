//未启用，未编写完整
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <Eigen/Geometry>

using boost::asio::serial_port;
using boost::asio::io_service;
using boost::asio::buffer;

class Placesend
{
public:
    Placesend(const std::string& port_name)
        : io_service_(), serial_port_(io_service_, port_name)
    {
        // 设置串口参数
        serial_port_.set_option(boost::asio::serial_port_base::baud_rate(115200)); // 波特率
        serial_port_.set_option(boost::asio::serial_port_base::character_size(8));  // 数据位
        serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));  // 停止位
        serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none)); // 奇偶校验
        serial_port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));  // 流控
    }

    // 向串口写入数据
    void sendData(const std::string& data)
    {
        boost::asio::write(serial_port_, buffer(data));
    }

private:
    io_service io_service_;
    serial_port serial_port_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_communication_node");
    ros::NodeHandle nh;

    // 创建串口通信对象，指定串口设备路径
    Placesend serial_comm("/dev/ttyUSB0");

    ros::Rate loop_rate(1);  // 设置循环频率

    while (ros::ok())
    {
        std::string data = "Hello from ROS";  // 你要发送的数据
        serial_comm.sendData(data);  // 发送数据

        ROS_INFO("Data sent: %s", data.c_str());

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

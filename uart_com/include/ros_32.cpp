#include "ros_32.h"

using namespace std;
using namespace boost::asio;

//串口发送接收相关常量、变量、共用体对象
const unsigned char ender[2] = {0x0d, 0x0a};//结束符 0x0d:回车 0x0a:换行
const unsigned char header[2] = {0x55, 0xaa};//帧头 0x55:帧头 0xaa:帧头
const unsigned char check_sum_header = 0x5a;//帧头校验和
const unsigned char check_sum_ender = 0xa5;//结束符校验和


//数据共用体
//机器人坐标信息
struct robot_coordinate
{
    float x;
    float y;
    float z;
};
//机器人姿态信息
struct robot_pose
{
    float roll;
    float pitch;
    float yaw;
};


//发送函数
//发送机器人坐标信息
void sendcoord(float x, float y, float z, boost::asio::serial_port& sp){
    unsigned char buf[18]={0};
    int i = 0;
    int len = 0;
    //消息头
    buf[i++] = header[0];
    buf[i++] = header[1];
    //数据长度
    len = 12;
    buf[i++] = len&0xff;
    buf[i++] = (len>>8)&0xff;
    //数据
    buf[i++] = (int)(x*1000)&0xff;
    buf[i++] = (int)(x*1000)>>8;
    buf[i++] = (int)(y*1000)&0xff;
    buf[i++] = (int)(y*1000)>>8;
    buf[i++] = (int)(z*1000)&0xff;
    buf[i++] = (int)(z*1000)>>8;
    //校验和
    unsigned char crc = getCrc8(buf, 18);
    buf[i++] = crc;
    //消息尾
    buf[i++] = ender[0];
    buf[i++] = ender[1];
    //发送数据
    boost::asio::write(sp, boost::asio::buffer(buf));
}

//发送机器人姿态信息
void sendpose(float roll, float pitch, float yaw, boost::asio::serial_port& sp){
    unsigned char buf[18]={0};
    int i = 0;
    int len = 0;
    //消息头
    buf[i++] = header[0];
    buf[i++] = header[1];
    //数据长度
    len = 12;
    buf[i++] = len&0xff;
    buf[i++] = (len>>8)&0xff;
    //数据
    buf[i++] = (int)(roll*1000)&0xff;
    buf[i++] = (int)(roll*1000)>>8;
    buf[i++] = (int)(pitch*1000)&0xff;
    buf[i++] = (int)(pitch*1000)>>8;
    buf[i++] = (int)(yaw*1000)&0xff;
    buf[i++] = (int)(yaw*1000)>>8;
    //校验和
    unsigned char crc = getCrc8(buf, 18);
    buf[i++] = crc;
    //消息尾
    buf[i++] = ender[0];
    buf[i++] = ender[1];
    //发送数据
    boost::asio::write(sp, boost::asio::buffer(buf));
}


//功能函数
//获取8位循环冗余校验码
unsigned char getCrc8(unsigned char *data, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *data++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
//串口初始化
void serialInit(boost::asio::serial_port& sp)
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}
//打开串口
void serialOpen(boost::asio::serial_port& sp){
    sp.open();
}
//关闭串口
void serialClose(boost::asio::serial_port& sp){
    sp.close();
}
//检测串口是否打开，返回true表示打开，false表示关闭
bool serialIsOpen(boost::asio::serial_port& sp){
    return sp.is_open();
}
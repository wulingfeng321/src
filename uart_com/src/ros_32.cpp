#include "../include/ros_32.h"

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

//串口相关设置
    //创建串口对象
    boost::asio::io_context iosev;//创建io_service对象
    //创建串口对象
    boost::asio::serial_port sp(iosev,"/dev/ttyUSB0");//创建串口对象


//发送函数
//发送测试数据“hello”
void sendTest(){
    unsigned char buf[10]= {0};
    int i = 0;
    //消息头
    buf[i++] = header[0];
    buf[i++] = header[1];
    //数据长度
    int len = 5;
    buf[i++] = len&0xff;
    //数据
    buf[i++] = 'h';
    buf[i++] = 'e';
    buf[i++] = 'l';
    buf[i++] = 'l';
    buf[i++] = 'o';
    //校验和
    unsigned char crc = getCrc8(buf, i);
    buf[i++] = crc;
    //消息尾
    buf[i++] = ender[0];
    buf[i++] = ender[1];
    //发送数据
    boost::asio::write(sp, boost::asio::buffer(buf));
    ROS_WARN("send hello complete");
}

//发送24字节数据信息，可容纳6个float数据,精度为precision1位小数和precision2位小数
//数据格式为：帧头  数据长度  数据  校验和  结束符
void send24byte(float& a, float& b, float& c,float& d, float& e, float& f,int precision1,int precision2){
    unsigned char buf[20]= {0};                                
    int i = 0;
    int len = 0;
    //消息头
    buf[i++] = header[0];
    buf[i++] = header[1];
    //数据长度
    len = 12;
    buf[i++] = len&0xff;
    //数据
    buf[i++] = (int)(a*precision1)&0xff;
    buf[i++] = (int)(a*precision1)>>8;
    buf[i++] = (int)(b*precision1)&0xff;
    buf[i++] = (int)(b*precision1)>>8;
    buf[i++] = (int)(c*precision1)&0xff;
    buf[i++] = (int)(c*precision1)>>8;
    buf[i++] = (int)(d*precision2)&0xff;
    buf[i++] = (int)(d*precision2)>>8;
    buf[i++] = (int)(e*precision2)&0xff;
    buf[i++] = (int)(e*precision2)>>8;
    buf[i++] = (int)(f*precision2)&0xff;
    buf[i++] = (int)(f*precision2)>>8;
    //校验和
    unsigned char crc = getCrc8(buf, i);
    buf[i++] = crc;
    //消息尾
    buf[i++] = ender[0];
    buf[i++] = ender[1];
    //发送数据
    boost::asio::write(sp, boost::asio::buffer(buf));
    ROS_INFO("send data complete: %f %f %f %f %f %f",a,b,c,d,e,f);
}


//接收函数
//接收测试数据“world”
void receiveTest(){
    unsigned char buf[10]= {0};
    int i = 0;
    //接收数据
    boost::asio::read(sp, boost::asio::buffer(buf, 10));
    //数据长度
    int len = buf[2];
    //数据
    for(i = 3; i < 8; i++){
        ROS_INFO("%c",buf[i]);
    }
    //校验和
    unsigned char crc = getCrc8(buf, 8);
    if(crc!= buf[8]){
        ROS_WARN("CRC ERROR!");
        return;
    }
    //消息尾
    if(buf[9]!= ender[0] || buf[10] != ender[1]){
        ROS_WARN("ENDER ERROR!");
        return;
    }
    ROS_WARN("receive world complete");
}

//接收24字节数据信息,精度为precision1位小数和precision2位小数，保存到六个float变量中
//数据格式为：帧头  数据长度  数据  校验和  结束符
void receive24byte(float& a, float& b, float& c,float& d, float& e, float& f,int precision1,int precision2){
    unsigned char buf[20]= {0};
    int i = 0;
    int len = 0;
    //接收数据
    boost::asio::read(sp, boost::asio::buffer(buf, 20));
    //数据长度
    len = buf[2];
    //数据
    a = (float)(buf[3]|(buf[4]<<8))/precision1;
    b = (float)(buf[5]|(buf[6]<<8))/precision1;
    c = (float)(buf[7]|(buf[8]<<8))/precision1;
    d = (float)(buf[9]|(buf[10]<<8))/precision2;
    e = (float)(buf[11]|(buf[12]<<8))/precision2;
    f = (float)(buf[13]|(buf[14]<<8))/precision2;
    //校验和
    unsigned char crc = getCrc8(buf, 15);
    if(crc!= buf[15]){
        ROS_WARN("CRC ERROR!");
        return;
    }
    //消息尾
    if(buf[16]!= ender[0] || buf[17]!= ender[1]){
        ROS_WARN("ENDER ERROR!");
        return;
    }
    ROS_INFO("receive data complete: %f %f %f %f %f %f",a,b,c,d,e,f);
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
void serialInit()
{
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));    
}
//打开串口
//void serialOpen(){
//    sp.open("/dev/ttyUSB0");
//}
//关闭串口
void serialClose(){
    sp.close();
}
//检测串口是否打开，返回true表示打开，false表示关闭
bool serialIsOpen(){
    return sp.is_open();
}
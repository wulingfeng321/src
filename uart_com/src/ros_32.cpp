#include "../include/ros_32.h"

using namespace std;

//串口发送接收相关常量、变量、共用体对象
const unsigned char ender[2] = {0x0d, 0x0a};//结束符 0x0d:回车 0x0a:换行
const unsigned char header[2] = {0x55, 0xaa};//帧头 0x55:帧头 0xaa:帧头
const unsigned char check_sum_header = 0x5a;//帧头校验和
const unsigned char check_sum_ender = 0xa5;//结束符校验和
std::string serial_name="/dev/pts/3"; //串口名称
//创建串口对象
boost::asio::io_context iosev;//创建io_service对象
//创建串口对象
boost::asio::serial_port sp(iosev, serial_name);//创建串口对象    
    

extern void serialInit(){
    //设置串口参数
    sp.set_option(boost::asio::serial_port_base::baud_rate(115200));
    sp.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    sp.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    sp.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    sp.set_option(boost::asio::serial_port_base::character_size(8));  
}

//发送函数
//发送测试数据“hello”
void sendTest(){
    unsigned char buf[11]= {0};
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
void send24byte(unsigned char* msg){
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
    buf[i++] = msg[0];
    buf[i++] = msg[1];
    buf[i++] = msg[2];
    buf[i++] = msg[3];
    buf[i++] = msg[4];
    buf[i++] = msg[5];
    buf[i++] = msg[6];
    buf[i++] = msg[7];
    buf[i++] = msg[8];
    buf[i++] = msg[9];
    buf[i++] = msg[10];
    buf[i++] = msg[11];
    //校验和
    unsigned char crc = getCrc8(buf, i);
    buf[i++] = crc;
    //消息尾
    buf[i++] = ender[0];
    buf[i++] = ender[1];
    //发送数据
    boost::asio::write(sp, boost::asio::buffer(buf));
    ROS_INFO("send data complete:");
}


//接收函数
//接收测试数据“world”
void receiveTest(unsigned char* msg){
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

//接收24字节数据信息，msg为接收到的字节数组
//数据格式为：帧头  数据长度  数据  校验和  结束符
unsigned char crcr;
int receive24byte(unsigned char* msg){
    unsigned char buf[20]= {0};
    int i = 0;
    int len = 0;
    //接收数据
    boost::asio::read(sp, boost::asio::buffer(buf, 20));
    //数据长度
    len = buf[2];
    //数据
    int k = 3;
    msg[k-3] = buf[k++];
    int 
    //校验和
    crcr = getCrc8(buf, 15);
    if(crcr!= buf[15]){
        ROS_WARN("CRC ERROR!");
        return -1;
    }
    //消息尾
    if(buf[16]!= ender[0] || buf[17]!= ender[1]){
        ROS_WARN("ENDER ERROR!");
        return -1;
    
    }
    ROS_INFO("receive data complete");
    return len;
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

//关闭串口
void serialClose()
{
    sp.close();
}

//数据处理函数
//机器人ros信息处理
//place_info拆分

void depart_place(const std_msgs::String::ConstPtr& msg) {
    //将消息中的数据保存到字符串中
    std::string serial_msg = msg->data;
    //信息切分
    std::stringstream ss(serial_msg);
    float a, b, c, d, e, f;
    ss >> a >> b >> c >> d >> e >> f;
    //转换数据,保留4位小数和3位小数
    int precision1 = 10000;
    int precision2 = 1000;
    int i = 0;
    msg_place[i++] = (int)(a*precision1)&0xff;
    msg_place[i++] = (int)(a*precision1)>>8;
    msg_place[i++] = (int)(b*precision1)&0xff;
    msg_place[i++] = (int)(b*precision1)>>8;
    msg_place[i++] = (int)(c*precision1)&0xff;
    msg_place[i++] = (int)(c*precision1)>>8;
    msg_place[i++] = (int)(d*precision2)&0xff;
    msg_place[i++] = (int)(d*precision2)>>8;
    msg_place[i++] = (int)(e*precision2)&0xff;
    msg_place[i++] = (int)(e*precision2)>>8;
    msg_place[i++] = (int)(f*precision2)&0xff;
    msg_place[i++] = (int)(f*precision2)>>8;
    //发送数据
    send24byte(msg_place);
}
//串口接受数据处理
//void serial_data_process(unsigned char* msg, int len,std::string message) {}
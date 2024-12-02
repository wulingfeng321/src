#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>
#include <thread>
#include <queue>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include "utils.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include "grab.h"
using namespace std;
using namespace Eigen;


int i = 0;
int main(int argc, char** argv) {
    ros::init(argc, argv, "sync_data_node");
    ros::NodeHandle nh;
    ROS_INFO("INIT UKF");
    EKF ekf(nh);
    ROS_INFO("INIT SYNC");
    ROS_INFO("INIT GRAB");
    Grabber grb;
    ros::Subscriber sub_imu = nh.subscribe("/livox/imu", 100, &Grabber::GrabImu, &grb);//imu
    ros::Subscriber sub_lido = nh.subscribe("/Odometry", 100, &Grabber::GrabLid, &grb);//lidar
    ros::Subscriber sub_camo = nh.subscribe("/orb_slam3/camera_pose", 100, &Grabber::GrabCam, &grb);//camera pose
    ros::Subscriber sub_img = nh.subscribe("/camera/infra1/image_rect_raw", 100, &Grabber::GrabImg, &grb);//camera image
    //ros::Subscriber sub_point = nh.subscribe("/cloud_registered", 100, &Grabber::GrabCam, &grb);// point cloud
    std::thread sync_thread(&Grabber::Sync, &grb, std::ref(ekf));//时间同步
    ros::spin();//阻塞
}
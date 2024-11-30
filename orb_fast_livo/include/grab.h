#ifndef GRABBER_H
#define GRABBER_H
#include <ekf.h>
#include <ros/ros.h>

#include <queue>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <mutex>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace Eigen;

// Forward declaration of SyncDataBuf class
class SyncDataBuf;

class Grabber
{
public:
    // Constructor that takes a pointer to SyncDataBuf
    Grabber(){};
    // Methods for grabbing different types of sensor data
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
    void GrabImg(const sensor_msgs::ImageConstPtr &imu_msg);

    void GrabLid(const nav_msgs::OdometryPtr &lid_msg);
    void GrabCam(const geometry_msgs::PoseStampedPtr &cam_msg);
    void updatePath(const VectorXd& state);

    // Synchronization method for EKF
    void Sync(EKF& ekf);

    // Method to convert ROS image message to OpenCV Mat
    cv::Mat GetImg(const sensor_msgs::ImageConstPtr &img_msg);

private:
    // Buffers to store incoming sensor messages
    std::queue<sensor_msgs::ImuConstPtr> mImuBuf; // Buffer for IMU data
    std::queue<nav_msgs::OdometryPtr> mLidBuf;    // Buffer for LiDAR odometry data
    std::queue<geometry_msgs::PoseStampedPtr> mCamBuf; // Buffer for camera pose data
    std::queue<sensor_msgs::ImageConstPtr> mImgBuf; // Buffer for camera pose data

};

#endif // GRABBER_H

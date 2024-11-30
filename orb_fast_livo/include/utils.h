#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Geometry> // 需要包含此头文件以使用旋转相关的功能

using namespace Eigen;
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
class Transformer{
public:
    Transformer() {
        // 相机坐标转化为雷达坐标平移向量
        Eigen::Vector3d L2CT(0.03296, -0.072, -0.054);
        Eigen::Vector3d C2LT(0.03296, -0.054, -0.072);
        // rgb外参
        Eigen::Vector3d C2IT(0.00096, -0.07729, 0.02788);
        Eigen::Vector3d I2CT(0.04696, -0.02788, -0.06071);
        // 定义一个绕 Z 轴旋转的四元数
        Quaterniond q1 = Quaterniond(AngleAxisd(-M_PI / 2 , Vector3d::UnitZ())); // 绕 Z 轴旋转 45°
        // 定义一个绕 Y 轴旋转的四元数
        Quaterniond q2 = Quaterniond(AngleAxisd(-M_PI / 2, Vector3d::UnitX())); // 绕 Y 轴旋转 45°
        // 组合旋转
        Quaterniond combined = q1 * q2;
        Matrix3d R = combined.toRotationMatrix();
        std::cout << "R: " << R << std::endl;
        // R << -1,  0,  0,
        //      0,  0,  -1,
        //      0,  -1,  0;
        // 初始化 mRT 为 4x4 齐次变换矩阵
        this->mL2CRT.setIdentity();  // 先将矩阵设为单位矩阵
        this->mC2LRT.setIdentity();  // 先将矩阵设为单位矩阵
        this->mC2IRT.setIdentity();  // 先将矩阵设为单位矩阵
        this->mI2CRT.setIdentity();  // 先将矩阵设为单位矩阵

        this->mL2CRT.block<3, 3>(0, 0) = R;  // 赋值旋转矩阵 R
        this->mL2CRT.block<3, 1>(0, 3) = L2CT;  // 赋值平移向量 T

        this->mC2LRT.block<3, 3>(0, 0) = R;  // 赋值旋转矩阵 R
        this->mC2LRT.block<3, 1>(0, 3) = C2LT;  // 赋值平移向量 T

        this->mC2IRT.block<3, 3>(0, 0) = R;  // 赋值旋转矩阵 R
        this->mC2IRT.block<3, 1>(0, 3) = C2IT;  // 赋值平移向量 T

        this->mI2CRT.block<3, 3>(0, 0) = R;  // 赋值旋转矩阵 R
        this->mI2CRT.block<3, 1>(0, 3) = I2CT;  // 赋值平移向量 T

    };
    Eigen::Vector4d LtoC(const Eigen::Vector3d& point) const;
    Eigen::Vector4d CtoL(const Eigen::Vector3d& points) const;
    Eigen::Vector4d CtoI(const Eigen::Vector3d& points) const;
    Eigen::Vector4d ItoC(const Eigen::Vector3d& points) const;

    Eigen::Matrix4d mL2CRT;
    Eigen::Matrix4d mC2LRT;
    Eigen::Matrix4d mC2IRT;
    Eigen::Matrix4d mI2CRT;

};
// class RGBPoint {
//     public:
//     RGBPoint() {
//         point_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
//         viewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
//         //创建3D窗口并添加点云
//         this->viewer->setBackgroundColor(0, 0, 0);
//         this->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
//         this->viewer->addPointCloud<pcl::PointXYZRGB>(this->point_cloud, "sample cloud");
//         this->viewer->initCameraParameters();
//     }
//     void projectGrayImageToPointCloud(const sensor_msgs::PointCloud2ConstPtr& point_msg,const cv::Mat& img, const nav_msgs::OdometryConstPtr& lid_odom,Transformer& TF);
//
//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud;
// };
#endif // UTILS_H
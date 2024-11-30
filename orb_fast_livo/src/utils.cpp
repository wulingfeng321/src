
#include "utils.h"

using namespace std;

Eigen::Vector4d Transformer::LtoC(const Eigen::Vector3d& point) const {
	Eigen::Vector4d point_homogeneous(point.x(), point.y(), point.z(), 1.0);
    // 使用变换矩阵进行转换
	Eigen::Vector4d Point_homogeneous = this->mL2CRT * point_homogeneous;
    // 返回前三个元素，即转换后的三维坐标
    return Point_homogeneous;
}

Eigen::Vector4d Transformer::CtoL(const Eigen::Vector3d& point) const {
	Eigen::Vector4d point_homogeneous(point.x(), point.y(), point.z(), 1.0);
	// 使用变换矩阵进行转换
	Eigen::Vector4d Point_homogeneous = this->mC2LRT * point_homogeneous;
	// 返回前三个元素，即转换后的三维坐标
	return Point_homogeneous;
}

Eigen::Vector4d Transformer::CtoI(const Eigen::Vector3d& point) const {
	Eigen::Vector4d point_homogeneous(point.x(), point.y(), point.z(), 1.0);
	// 使用变换矩阵进行转换
	Eigen::Vector4d Point_homogeneous = this->mC2IRT * point_homogeneous;
	// 返回前三个元素，即转换后的三维坐标
	return Point_homogeneous;
}

Eigen::Vector4d Transformer::ItoC(const Eigen::Vector3d& point) const {
	Eigen::Vector4d point_homogeneous(point.x(), point.y(), point.z(), 1.0);
	// 使用变换矩阵进行转换
	Eigen::Vector4d Point_homogeneous = this->mI2CRT * point_homogeneous;
	// 返回前三个元素，即转换后的三维坐标
	return Point_homogeneous;
}


// void RGBPoint::projectGrayImageToPointCloud(
//     const sensor_msgs::PointCloud2ConstPtr& point_msg,
//     const cv::Mat& img,
//     const nav_msgs::OdometryConstPtr& lid_odom,
//     Transformer& TF) {
//
//     // 将 PointCloud2 转换为 PCL 点云格式
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr point_xyz(new pcl::PointCloud<pcl::PointXYZ>);
// 	pcl::fromROSMsg(*point_msg, *point_xyz);
// 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
// 	pcl::copyPointCloud(*point_xyz, *point_rgb);
//     double fx = 382.613;
//     double fy = 382.613;
//     double cx = 320.183;
//     double cy = 236.455;
//     Eigen::MatrixXd camera_par(3, 4);
//     camera_par <<914.4378051757812,  0,  636.219970703125,  0,
//                  0,  912.5625,  358.07366943359375,  0,
//                  0,   0,   1,  0; // 这是4列的形式
//
//     Eigen::Vector3d word_h;
// 	Eigen::Vector3d p_result;
// 	int rows = img.rows;
// 	int cols = img.cols;
//
// 	double p_u, p_v, p_w;//pics_uv1;(u for cols, v for lines!!!)
// 	double c_x, c_y, c_z, c_i;//clouds_xyz、intensity;
// 	for (int nIndex = 0; nIndex < point_rgb->points.size(); nIndex++) {
// 		double c_x = point_rgb->points[nIndex].x;
// 		double c_y = point_rgb->points[nIndex].y;
// 		double c_z = point_rgb->points[nIndex].z;
// 		word_h << c_x, c_y, c_z;
// 		p_result = camera_par * TF.ItoC(word_h);
//
// 		p_w = p_result(2);
// 		p_u = (int)((p_result(0)) / p_w);
// 		p_v = (int)((p_result(1)) / p_w);
//
//
// 		if (p_u >= 0 && p_u < cols-1 && p_v >= 0 && p_v < rows-1 && p_w > 0) {
// 			float r = img.at<cv::Vec3b>(p_v, p_u)[2];
// 			float g = img.at<cv::Vec3b>(p_v, p_u)[1];
// 			float b = img.at<cv::Vec3b>(p_v, p_u)[0];
// 			// 使用新颜色
// 			point_rgb->points[nIndex].r = r;
// 			point_rgb->points[nIndex].g = g;
// 			point_rgb->points[nIndex].b = b;
// 		} else {
// 			// 保持旧颜色或设置为白色
// 			if (this->point_cloud->points.size() > nIndex) {
// 				// 保持之前的颜色
// 				point_rgb->points[nIndex].r = this->point_cloud->points[nIndex].r;
// 				point_rgb->points[nIndex].g = this->point_cloud->points[nIndex].g;
// 				point_rgb->points[nIndex].b = this->point_cloud->points[nIndex].b;
// 			} else {
// 				// 如果没有旧点，则设置为白色
// 				point_rgb->points[nIndex].r = 255;
// 				point_rgb->points[nIndex].g = 255;
// 				point_rgb->points[nIndex].b = 255;
// 			}
// 		}
// 	}
//
// 	// 合并新点云和旧点云
// 	*this->point_cloud += *point_rgb;
// 	// 提取平移信息（位置）
// 	double pos_x = lid_odom->pose.pose.position.x;
// 	double pos_y = lid_odom->pose.pose.position.y;
// 	double pos_z = lid_odom->pose.pose.position.z;
//
// 	// 提取旋转信息（四元数）
// 	double orientation_x = lid_odom->pose.pose.orientation.x;
// 	double orientation_y = lid_odom->pose.pose.orientation.y;
// 	double orientation_z = lid_odom->pose.pose.orientation.z;
// 	double orientation_w = lid_odom->pose.pose.orientation.w;
//
// 	// 将四元数转换为 Eigen 四元数类型
// 	Eigen::Quaternionf quat(orientation_w, orientation_x, orientation_y, orientation_z);
//
// 	// 将四元数转换为旋转矩阵
// 	Eigen::Matrix3f rotation_matrix = quat.toRotationMatrix();
//
// 	// 创建一个 4x4 的变换矩阵
// 	Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
//
// 	// 将旋转部分（3x3）和位移部分（3x1）填入变换矩阵
// 	transform_matrix.block<3,3>(0,0) = rotation_matrix; // 旋转部分
// 	transform_matrix(0,3) = pos_x; // 平移 X
// 	transform_matrix(1,3) = pos_y; // 平移 Y
// 	transform_matrix(2,3) = pos_z; // 平移 Z
// 	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
// 	transform.matrix() = transform_matrix; // 赋值变换矩阵
// 	// 主循环
// 	// viewer->removePointCloud("sample cloud");
//     this->viewer->addCoordinateSystem(0.1, transform);
// 	viewer->updatePointCloud(this->point_cloud, "sample cloud");
// 	this->viewer->spinOnce(1000);
//
// }
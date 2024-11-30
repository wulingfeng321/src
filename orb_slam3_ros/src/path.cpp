#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <vector>

// 插值函数：简单线性插值或样条插值
std::vector<Eigen::Vector3d> interpolatePath(const std::vector<Eigen::Vector3d>& keyframe_positions, int num_points) {
    std::vector<Eigen::Vector3d> interpolated_path;

    if (keyframe_positions.size() < 2) {
        ROS_WARN("关键帧数量不足，无法进行插值。");
        return interpolated_path;
    }

    for (size_t i = 0; i < keyframe_positions.size() - 1; ++i) {
        Eigen::Vector3d start = keyframe_positions[i];
        Eigen::Vector3d end = keyframe_positions[i + 1];
        for (int j = 0; j < num_points; ++j) {
            double t = static_cast<double>(j) / num_points;
            Eigen::Vector3d interpolated_point = (1 - t) * start + t * end;
            interpolated_path.push_back(interpolated_point);
        }
    }

    interpolated_path.push_back(keyframe_positions.back());  // 最后一个点直接加入
    return interpolated_path;
}

// 发布拟合路径
void publishFittedPath(const std::vector<Eigen::Vector3d>& path, ros::Publisher& path_pub) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";  // 假设路径在"map"坐标系
    path_msg.header.stamp = ros::Time::now();

    for (const auto& point : path) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "map";
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = point.x();
        pose_stamped.pose.position.y = point.y();
        pose_stamped.pose.position.z = point.z();
        pose_stamped.pose.orientation.w = 1.0;  // 假设不需要朝向信息

        path_msg.poses.push_back(pose_stamped);
    }

    path_pub.publish(path_msg);
    ROS_INFO("发布了拟合路径");
}

void kfMarkersCallback(const visualization_msgs::Marker::ConstPtr& msg, ros::Publisher& path_pub) {
    std::vector<Eigen::Vector3d> keyframe_positions;

    // 提取关键帧的位置信息
    // 提取关键帧的位置信息，遍历 points 数组
    for (const auto& point : msg->points) {
        Eigen::Vector3d position(point.x, point.y, point.z);
        keyframe_positions.push_back(position);
    }
    // 进行路径插值和发布
    if (keyframe_positions.size() > 1) {
        std::vector<Eigen::Vector3d> fitted_path = interpolatePath(keyframe_positions, 10);
        publishFittedPath(fitted_path, path_pub);
        ROS_INFO("插入关键帧并发布路径");
    } else {
//        ROS_WARN("关键帧数量不足，无法拟合路径。");
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "kf_path_fitter");
    ros::NodeHandle nh;

    // 发布路径的Publisher
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/fitted_path", 10);

    // 订阅关键帧数据
    ros::Subscriber kf_sub = nh.subscribe<visualization_msgs::Marker>("/orb_slam3/kf_markers", 10,
        boost::bind(&kfMarkersCallback, _1, boost::ref(path_pub)));

    ros::spin();
    return 0;
}

#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <iostream>
#include <nav_msgs/Odometry.h> // Ensure you have ROS installed to use this
#include <Eigen/Geometry> // For Quaternion to Euler conversion
#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace Eigen;
using namespace std;
using namespace std::chrono;

class EKF {
public:
    EKF(ros::NodeHandle& nh);  // Constructor

    // Function to predict the next state
    void predict(const VectorXd &imu_data, double dt);

    // Function to update the state with sensor measurements
    void update(nav_msgs::Odometry::ConstPtr &lid_msg);
    void publishPose();
    void broadcastTransform();

    // Function to print the estimated state
    void printState() const;

    // Function to retrieve the estimated state
    VectorXd getState();

private:
    // Private member variables
    VectorXd x_hat; // State estimate
    MatrixXd P;     // State covariance estimate
    MatrixXd F;     // State transition matrix
    MatrixXd Q;     // Process noise covariance
    MatrixXd R;     // Measurement noise covariance
    high_resolution_clock::time_point start_time;
    int prediction_count;
    ros::NodeHandle nh_;                       // ROS节点句柄
    ros::Publisher pose_pub_;                  // ROS发布器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; // TF广播器
    // Function to update the state transition matrix
    void updateF(double dt);
};

#endif // EKF_H

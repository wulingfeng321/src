#include "ekf.h"
EKF::EKF( ros::NodeHandle& nh) : nh_(nh) {
    // 初始化状态向量
    x_hat = VectorXd(9);
    x_hat << 0, 0, 0, 0, 0, 0, 0, 0, 0; // [x, y, z, vx, vy, vz, yaw, pitch, roll]
    // 初始化协方差矩阵
    P = MatrixXd::Identity(9, 9) * 0.1;

    // 初始化过程噪声和观测噪声协方差
    Q = MatrixXd::Identity(9, 9) * 0.1; // 过程噪声
    R = MatrixXd::Identity(6, 6) * 0.01; // 观测噪声（仅位置和欧拉角）

    // 状态转移矩阵 F（线性近似）
    F = MatrixXd::Identity(9, 9);

    // 初始化计时相关变量
    start_time = high_resolution_clock::now();//high_resolution_clock::now()返回当前函数运行时间戳
    prediction_count = 0;

    // 初始化发布器
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("odom", 10000);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(); // 创建 TF 广播器
}
void EKF::publishPose() {
    // 创建 PoseStamped 消息
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world"; // 设置坐标系为“world”

    // 填充位置
    pose_msg.pose.position.x = x_hat(0);
    pose_msg.pose.position.y = x_hat(1);
    pose_msg.pose.position.z = x_hat(2);

    // 填充四元数（假设姿态以欧拉角表示，需转换为四元数）
    Quaterniond q;
    q = AngleAxisd(x_hat(6), Vector3d::UnitZ()) * // yaw
        AngleAxisd(x_hat(7), Vector3d::UnitY()) * // pitch
        AngleAxisd(x_hat(8), Vector3d::UnitX()); // roll

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    // 发布消息
    pose_pub_.publish(pose_msg);
}

void EKF::predict(const VectorXd &imu_data, double dt) {
    // 使用 IMU 数据更新状态预测
    // imu_data = [ax, ay, az, wx, wy, wz]
    VectorXd u(6);
    u << imu_data; // 使用 IMU 数据作为输入
    // 使用简单的模型进行状态更新
    x_hat.head(3) += x_hat.segment(3, 3) * dt; // 更新位置
    x_hat.segment(3, 3) += u.head(3) * dt; // 更新速度
    x_hat.segment(6, 3) += u.tail(3) * dt; // 更新姿态
    // 增加预测计数
    prediction_count++;

    // 计算经过的时间
    auto now = high_resolution_clock::now();
    auto duration = duration_cast<seconds>(now - start_time);
    if (duration.count() >= 1) {
        // 每过一秒输出预测数量
        cout << "发布频率: " << prediction_count << "Hz"<< endl;
        // 重置计数器和开始时间
        prediction_count = 0;
        start_time = now;
    }
    // 发布预测的位姿
    publishPose();

    // 广播 TF
    broadcastTransform();
    // 显示预测的状态
    cout << "预测状态：" << x_hat.head(3).transpose() << " "
         << x_hat.segment(6, 3).transpose() << endl;
    updateF(dt);
    // 更新协方差预测
    P = F * P * F.transpose() + Q;
}

void EKF::broadcastTransform() {
    // 创建变换消息
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = "world"; // 父坐标系
    transform_msg.child_frame_id = "robot"; // 子坐标系

    // 设置位置
    transform_msg.transform.translation.x = x_hat(0);
    transform_msg.transform.translation.y = x_hat(1);
    transform_msg.transform.translation.z = x_hat(2);

    // 设置姿态（四元数）
    Quaterniond q;
    q = AngleAxisd(x_hat(6), Vector3d::UnitZ()) *
        AngleAxisd(x_hat(7), Vector3d::UnitY()) *
        AngleAxisd(x_hat(8), Vector3d::UnitX());

    transform_msg.transform.rotation.x = q.x();
    transform_msg.transform.rotation.y = q.y();
    transform_msg.transform.rotation.z = q.z();
    transform_msg.transform.rotation.w = q.w();

    // 广播变换
    tf_broadcaster_->sendTransform(transform_msg);
}
void EKF::updateF(double dt) {
    F << 1, 0, 0, dt, 0, 0, 0, 0, 0,
         0, 1, 0, 0, dt, 0, 0, 0, 0,
         0, 0, 1, 0, 0, dt, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, dt, 0,
         0, 0, 0, 0, 0, 0, 0, 1, dt,
         0, 0, 0, 0, 0, 0, 0, 0, 1;
}
void EKF::update(nav_msgs::Odometry::ConstPtr &lid_msg) {
    // 观测值：位置和欧拉角姿态
    VectorXd z(6);
    z.head(3) << lid_msg->pose.pose.position.x,
                 lid_msg->pose.pose.position.y,
                 lid_msg->pose.pose.position.z;

    // 将四元数从消息转换为欧拉角（假设ROS中的姿态消息为四元数形式）
    Quaterniond q_meas(lid_msg->pose.pose.orientation.w,
                       lid_msg->pose.pose.orientation.x,
                       lid_msg->pose.pose.orientation.y,
                       lid_msg->pose.pose.orientation.z);
    Vector3d euler_meas = q_meas.toRotationMatrix().eulerAngles(2, 1, 0); // 转换为Yaw, Pitch, Roll
    cout << "slam状态" << z.head(3).transpose() << " " << euler_meas.transpose() << endl;
    z.tail(3) = euler_meas;

    // 计算位置和姿态观测残差
    VectorXd y(6);
    y.head(3) = z.head(3) - x_hat.head(3);           // 位置观测残差
    y.tail(3) = z.tail(3) - x_hat.segment(6, 3);     // 姿态观测残差

    // 观测矩阵 H：将位置和姿态观测矩阵结合
    MatrixXd H = MatrixXd::Zero(6, 9);
    H.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3); // 位置部分
    H.block<3, 3>(3, 6) = MatrixXd::Identity(3, 3); // 姿态部分

    // 计算卡尔曼增益
    MatrixXd S = H * P * H.transpose() + R;          // 残差协方差
    MatrixXd K = P * H.transpose() * S.inverse();    // 卡尔曼增益

    // 更新状态估计
    x_hat += K * y;

    // 更新协方差矩阵
    P = (MatrixXd::Identity(9, 9) - K * H) * P;

    cout << "更新后的状态：" << x_hat.head(3).transpose() << " " << x_hat.segment(6, 3).transpose() << endl;
}

void EKF::printState() const {
    cout << "Estimated state:\n" << x_hat.transpose() << endl;
}

VectorXd EKF::getState() {
    return x_hat;
}


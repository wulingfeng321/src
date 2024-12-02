#include "grab.h"


std::vector<VectorXd> estimated_states;

bool first = false;
double previousTime = 0;
double currentTime = 0;
double delta_t = 0;
std::mutex m;

void Grabber::Sync(EKF& ekf)
{
    const double maxTimeDiff = 0.1;// 最大时间差
    std::cout << std::fixed << std::setprecision(8);

    while(true)
    {
        double tLid = 0, tCam = 0;
        // 雷达相机位姿时间同步
        if (!mLidBuf.empty()&&/*!mCamBuf.empty()&&*/!mImuBuf.empty())
        {
            m.lock();
            tLid = mLidBuf.front()->header.stamp.toSec();
            //tCam = mCamBuf.front()->header.stamp.toSec();
            m.unlock();

            /*while((tLid-tCam)>maxTimeDiff && mCamBuf.size()>1)
            {
                m.lock();
                mCamBuf.pop();
                tCam = mCamBuf.front()->header.stamp.toSec();
                m.unlock();

            }

            while((tCam-tLid)>maxTimeDiff && mLidBuf.size()>1)
            {
                m.lock();
                mLidBuf.pop();
                tLid = mLidBuf.front()->header.stamp.toSec();
                m.unlock();

            }

            if((tLid-tCam)>maxTimeDiff || (tCam-tLid)>maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }*/

            if(tLid > mImuBuf.back()->header.stamp.toSec())
                continue;

            m.lock();
            
            // 雷达相机位姿时间同步完成
            ros::Time msg_time = mLidBuf.front()->header.stamp;
            nav_msgs::Odometry::ConstPtr lido_msg = mLidBuf.front();
            mLidBuf.pop();
            //mCamBuf.pop();
            m.unlock();

            if(!mImuBuf.empty())
            {
                // imu数据同步完成
                m.lock();
                VectorXd imu(6);

                bool sync_imu = false;
                while(!mImuBuf.empty() && mImuBuf.front()->header.stamp.toSec()<=tLid)
                {
                    previousTime = currentTime;
                    currentTime = mImuBuf.front()->header.stamp.toSec();
                    // 首次判断
                    if (!first) {
                        first = true;
                        mImuBuf.pop();
                        m.unlock();
                        continue;
                    }

                    // 估计状态
                    if(currentTime >= previousTime && currentTime <= msg_time.toSec()) {
                        delta_t = currentTime - previousTime;
                        // std::cout << "imu时间差:" << delta_t << std::endl;

                        sensor_msgs::ImuConstPtr imu_data = mImuBuf.front();// 读取imu数据
                        imu <<  imu_data->linear_acceleration.x,
                                    imu_data->linear_acceleration.y,
                                    imu_data->linear_acceleration.z,
                                    imu_data->angular_velocity.x,
                                    imu_data->angular_velocity.y,
                                    imu_data->angular_velocity.z;
                        ekf.predict(imu, delta_t); // 使用IMU数据进行状态预测
                        sync_imu = true;
                        // 保存估计状态
                        estimated_states.push_back(ekf.getState());
                        mImuBuf.pop();
                        m.unlock();

                    }
                }
                if(sync_imu) {
                    m.lock();
                    ekf.update(lido_msg);
                    Vector3d position;
                    position << lido_msg->pose.pose.position.x,lido_msg->pose.pose.position.y,lido_msg->pose.pose.position.z;
                    Quaterniond orientation;
                    orientation.coeffs() << lido_msg->pose.pose.orientation.w, lido_msg->pose.pose.orientation.x, lido_msg->pose.pose.orientation.y, lido_msg->pose.pose.orientation.z;
                    m.unlock();
                }
                m.unlock();

                // cout << "同步时间:" << msg_time.toSec() << endl;
            }
        }
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

// 雷达数据压入缓冲区
void Grabber::GrabLid(const nav_msgs::OdometryPtr &lid_msg)
{
    m.lock();
    this->mLidBuf.push(lid_msg);
    m.unlock();

}
// 相机数据压入缓冲区
void Grabber::GrabCam(const geometry_msgs::PoseStampedPtr &cam_msg)
{
    m.lock();
    this->mCamBuf.push(cam_msg);
    m.unlock();

}

// imu数据压入缓冲区
void Grabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    m.lock();
    this->mImuBuf.push(imu_msg);
    m.unlock();

}

// 图像数据压入缓冲区
void Grabber::GrabImg(const sensor_msgs::ImageConstPtr &img_msg)
{
    m.lock();
    this->mImgBuf.push(img_msg);
    m.unlock();

}

// Method to convert ROS image message to OpenCV Mat
//将 ROS 图像消息转换为 OpenCV Mat
cv::Mat Grabber::GetImg(const sensor_msgs::ImageConstPtr &img_msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
        // 将 RGB 图像消息转换为 OpenCV BGR 格式图像
        // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

        // 获取 BGR 图像
        cv::Mat bgr_image = cv_ptr->image;
        return cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}
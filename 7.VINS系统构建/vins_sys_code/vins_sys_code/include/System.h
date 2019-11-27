#pragma once

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include <fstream>
#include <condition_variable>

// #include <cv.h>
// #include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "feature_tracker.h"

//采用IMU的消息
//imu for vio
struct IMU_MSG
{
    double header;                          //时间戳
    Eigen::Vector3d linear_acceleration;    //线加速度向量
    Eigen::Vector3d angular_velocity;       //角速度向量
};

//实例化一个指向结构体IMU_MSG的智能指针
typedef std::shared_ptr<IMU_MSG const> ImuConstPtr;    

//图像数据的时间戳
//image for vio
struct IMG_MSG
{
    double header;                         //时间戳
    vector<Vector3d> points;               //包含该图像特征点对应的三维点坐标（3D）
    vector<int> id_of_point;               //特征点对应的ID号
    vector<float> u_of_point;              //特征点对应的u坐标
    vector<float> v_of_point;              //特征点对应的v坐标
    vector<float> velocity_x_of_point;     //特征点对应的u-axis光流的速度
    vector<float> velocity_y_of_point;     //特征点对应的v-axis光流的速度
};

//实例化一个指向结构体IMG_MSG的智能指针
typedef std::shared_ptr<IMG_MSG const> ImgConstPtr;

class System
{
public:
    //系统的启动文件（配置文件）
    System(std::string sConfig_files);                            

    ~System();
    
    void PubFeatureData(double dStampSec, const vector<int> &feature_id, const vector<Vector2d> &feature, const vector<Vector2d> &observation, std::vector<Vector2d> &featureVelocity);

    void PubImageData(double dStampSec, cv::Mat &img);

    void PubImuData(double dStampSec, const Eigen::Vector3d &vGyr,
                    const Eigen::Vector3d &vAcc);
    
    // thread: visual-inertial odometry
    void ProcessBackEnd();
    void Draw();

private:
    //feature tracker
    std::vector<uchar> r_status;            //
    std::vector<float> r_err;               //
    // std::queue<ImageConstPtr> img_buf;

    // ros::Publisher pub_img, pub_match;
    // ros::Publisher pub_restart;

    FeatureTracker trackerData[NUM_OF_CAM]; //实例化NUM_OF_CAM个特征追踪的对象
    double first_image_time;                //第一次图像时间  
    int pub_count = 1;                      //发布的图像数量
    bool first_image_flag = true;           //第一次图像的标志
    double last_image_time = 0;             //上一次图像时间
    bool init_pub = 0;                      //skip the first image; since no optical speed on frist image
    
    //estimator
    Estimator estimator;                    //后端非线性状态估计器

    std::condition_variable con;            //条件变量
    double current_time = -1;               //当前时间
    std::queue<ImuConstPtr> imu_buf;        //IMU队列
    std::queue<ImgConstPtr> feature_buf;    //图像队列
    // std::queue<PointCloudConstPtr> relo_buf;
    int sum_of_wait = 0;                    //???????

    //线程互斥锁
    std::mutex m_buf;   
    std::mutex m_state;
    std::mutex i_buf;
    std::mutex m_estimator;

    //pvq ba bg 
    double latest_time;                  
    Eigen::Vector3d tmp_P;
    Eigen::Quaterniond tmp_Q;
    Eigen::Vector3d tmp_V;
    Eigen::Vector3d tmp_Ba;
    Eigen::Vector3d tmp_Bg;
    Eigen::Vector3d acc_0;
    Eigen::Vector3d gyr_0;
    
    //初始化参数
    bool init_feature = 0;
    bool init_imu = 1;
    double last_imu_t = 0;

    //输入psoe
    std::ofstream ofs_pose;
    std::vector<Eigen::Vector3d> vPath_to_draw;

    bool bStart_backend;                  //开启后端优化标志位

    //得到IMU 和 img的对齐向量   vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>>>
    std::vector<std::pair<std::vector<ImuConstPtr>, ImgConstPtr>> getMeasurements();
public:
    vector<Vector3d> real_poses;

};
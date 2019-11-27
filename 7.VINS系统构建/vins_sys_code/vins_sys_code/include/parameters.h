#pragma once

// #include <ros/ros.h>
#include <vector>                             
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"               //矩阵四元素变化   
// #include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>
#include <fstream>

//feature tracker
// extern int ROW;
// extern int COL;
const int NUM_OF_CAM = 1;                 //相机的个数:单目

extern int FOCAL_LENGTH;                  //焦距
extern std::string IMAGE_TOPIC;           //相机的图像话题
extern std::string IMU_TOPIC;             //IMU话题
extern std::string FISHEYE_MASK;          //淹摸mask:https://www.cnblogs.com/skyfsm/p/6894685.html
extern std::vector<std::string> CAM_NAMES;//保存相机的名字
extern int MAX_CNT;                       //提取的最大的光流
extern int MIN_DIST;                      //两个特征点之间最小距离
// extern int WINDOW_SIZE;
extern int FREQ;                          //图像发布的帧率
extern double F_THRESHOLD;                //求取基础矩阵的时候阙值
extern int SHOW_TRACK;                    //显示追踪，发布图像话题
extern bool STEREO_TRACK;                 //双目还是单目
extern int EQUALIZE;                      //是否采用均衡化，当图像过暗和过亮
extern int FISHEYE;                       //采用的是否为fisheye相机
extern bool PUB_THIS_FRAME;               //是否发布当前帧

//estimator

// const double FOCAL_LENGTH = 460.0;
const int WINDOW_SIZE = 10;              //滑动窗口的大小
// const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;               //特征点
//#define UNIT_SPHERE_ERROR

extern double INIT_DEPTH;                //初始化深度
extern double MIN_PARALLAX;              //最小化平均视差
extern int ESTIMATE_EXTRINSIC;           //估计外参：  0   1:相机标定啦    2 ：没有外参

extern double ACC_N, ACC_W;              //加速度的白噪声和随机游走的不确定度
extern double GYR_N, GYR_W;              //陀螺仪的白噪声和随机游走的不确定度

extern std::vector<Eigen::Matrix3d> RIC; //存放相机相对于IMU的姿态
extern std::vector<Eigen::Vector3d> TIC; //存放相机相对于IMU的位移
extern Eigen::Vector3d G;                //重力向量

extern double BIAS_ACC_THRESHOLD;        //加速度计偏置的阈值
extern double BIAS_GYR_THRESHOLD;        //陀螺仪的偏置的阈值
extern double SOLVER_TIME;               //求解时间
extern int NUM_ITERATIONS;               //最大的迭代次数
extern std::string EX_CALIB_RESULT_PATH; //外参标定结果的路径
extern std::string VINS_RESULT_PATH;     //VINS结果路径
extern std::string IMU_TOPIC;            //IMU话题名
extern double TD;                        //IMU和相机的时间戳差
extern double TR;                        //全局快门时间和滚动快门，该变量为滚动快门时间
extern int ESTIMATE_TD;                  //估算的IMU和相机的时间戳差
extern int ROLLING_SHUTTER;              //是否为滚动快门
extern double ROW, COL;                  //图像长和宽

// void readParameters(ros::NodeHandle &n);
void readParameters(std::string config_file);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};

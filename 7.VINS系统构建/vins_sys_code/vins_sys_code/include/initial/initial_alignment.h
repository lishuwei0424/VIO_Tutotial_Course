#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

#include "../factor/integration_base.h"
#include "../utility/utility.h"
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

/****************************************************************************************************************
ImageFrame 图像帧类
图像帧类可由图像帧的特征点与时间戳构造，
此外还保存了位姿Rt，预积分对象pre_integration，是否是关键帧。
****************************************************************************************************************/
class ImageFrame
{
public:
    ImageFrame(){};
    ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &_points, double _t) : t{_t}, is_key_frame{false}
    {
        points = _points;
    };
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> points;

    double t;
    Matrix3d R;
    Vector3d T;
    IntegrationBase *pre_integration;
    bool is_key_frame;
};

/*****************************************************************************************************************
bool Estimator::visualInitialAlign() 视觉惯性联合初始化
该函数主要实现了陀螺仪的偏置校准(加速度偏置没有处理)，计算速度V[0:n]、重力g、尺度s。
同时更新了Bgs后，IMU测量量需要repropagate；得到尺度s和重力g的方向后，需更新所有图像帧在世界坐标系下的Ps、Rs、Vs。
*****************************************************************************************************************/
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d *Bgs, Vector3d &g, VectorXd &x);

//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t) : Rwc(R), qwc(R), twc(t){};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv; // 这帧图像观测到的特征坐标
};
int main()
{
    std::cout << "1" << std::endl;
    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for (int n = 0; n < poseNums; ++n)
    {
        double theta = n * 2 * M_PI / (poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R, t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);

    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i)
    {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        camera_pose[i].uv = Eigen::Vector2d(x / z, y / z);
    }

    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est; // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */

    //主要思想：相机多帧观测对应空间在一点
    Eigen::Matrix4d D;
    Eigen::MatrixXd P(2 * (end_frame_id - start_frame_id), 4);

    for (int i = 3, j = 0; i < poseNums; i++)
    {
        Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
        Tcw.rotate(camera_pose[i].Rwc.transpose());
        Tcw.pretranslate(-camera_pose[i].Rwc.transpose() * camera_pose[i].twc);

        P.block(j, 0, 1, 4) = camera_pose[i].uv.x() * Tcw.matrix().row(2) - Tcw.matrix().row(0);
        P.block(j + 1, 0, 1, 4) = camera_pose[i].uv.y() * Tcw.matrix().row(2) - Tcw.matrix().row(1);
        j += 2;
    }

    D = P.transpose() * P;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(D, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    P_est << (U.rightCols(1) * U.rightCols(1).bottomRows(1).inverse()).topRows(3);

    /* your code end */

    std::cout << "ground truth: \n"
              << Pw.transpose() << std::endl;
    std::cout << "your result: \n"
              << P_est.transpose() << std::endl;
    return 0;
}

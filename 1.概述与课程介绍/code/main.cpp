/*参考博文eigen3
  https://www.cnblogs.com/goingupeveryday/p/5699053.html
 */

#include <iostream>

//using eigen3
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;


int main() {
    //随机生成随机旋转矩阵
    Eigen::Vector3d   rot_axis=Eigen::Vector3d::Random();
    rot_axis.normalize();

    Eigen::AngleAxisd  rot_angle_axis(M_PI/4,rot_axis);
    Eigen::Matrix3d  R=rot_angle_axis.toRotationMatrix();
    cout<<"rotation  matrix  R before  update"<<endl<<R<<endl;

    Eigen::Vector3d w(0.01,0.02,0.03);
    double  robot_angle=w.norm();
    Eigen::AngleAxisd  w_rot_angle_axis(robot_angle,w.normalized());

    //update R using w.toRotationMatrix()
    Eigen::Matrix3d     w_rot_matrix=w_rot_angle_axis.toRotationMatrix();
    R=R*w_rot_matrix;
    cout<<"rotation  matrix R after  update using matrix"<<endl<<R<<endl;

    //update R using q
    Eigen::Quaterniond  q(rot_angle_axis);
    Eigen::Quaterniond  q_w(1,0.5*w(0),0.5*w(1),0.5*w(2));
    q=q*q_w;

    q.normalize();
    cout<<"rotation  matrix R  after  update using q"<<endl<<q.toRotationMatrix()<<endl;

    return 0;
}

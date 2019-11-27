#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
// string sConfig_path = "../config/";
string sConfig_file = "../config/";
string sData_path = "/home/vslam/VIO_Tutorial/7.vins_system/vins_sys_code/data/";
string feature_file = "/keyframe/"; //文件夹下有每一帧cam提取的pose
//string imu_data = "imu_pose.txt";   // imu_raw data
string imu_data = "imu_pose_noise.txt";

std::shared_ptr<System> pSystem;

//将IMU数据载入system
void PubImuData()
{
	string sImu_data_file = sData_path + imu_data;
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;

	double lastTime;

	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data　every line
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		pSystem->PubImuData(dStampNSec, vGyr, vAcc);

		lastTime = dStampNSec;
		usleep(4500 * nDelayTimes);
	}
	fsImu.close();
}

void PubRealData()
{
	string sRealData_file = sData_path + "cam_pose.txt";
	ifstream fsPose;
	fsPose.open(sRealData_file.c_str());
	if (!fsPose.is_open())
	{
		cerr << "Failed to open realPose file! " << sRealData_file << endl;
		return;
	}
	std::string sPose_line;
	Vector3d current_pose;
	double dStampNSec = 0.0;

	while (std::getline(fsPose, sPose_line) && !sPose_line.empty())
	{
		std::istringstream ssPoseData(sPose_line);
		ssPoseData >> dStampNSec >> current_pose.x() >> current_pose.y() >> current_pose.z();

		pSystem->real_poses.push_back(current_pose);
		usleep(50000 * nDelayTimes);
	}
}

//将image载入system
void PubImageData()
{
	string sImage_file = sData_path + "image_filename.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str()); //所有图像特征的文件名列表　600个
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	while (std::getline(fsImage, sImage_line) && !sImage_line.empty()) // 读取每一行，每一行都是包含一副图片所有的特征点文件名
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;

		string imagePath = sData_path + sImgFileName;

		//读取每个camera提取的特征点
		ifstream featuresImage;
		featuresImage.open(imagePath.c_str());
		if (!featuresImage.is_open())
		{
			cerr << "Failed to open features file! " << imagePath << endl;
			return;
		}
		std::string featuresImage_line;
		std::vector<int> feature_id;
		int ids = 0;

		std::vector<Vector2d> featurePoint;
		std::vector<Vector2d> observation_feature;
		std::vector<Vector2d> featureVelocity;
		static double lastTime;
		static std::vector<Vector2d> lastfeaturePoint(50);

		cv::Mat show_img(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));

		while (std::getline(featuresImage, featuresImage_line) && !featuresImage_line.empty())
		{
			Vector2d current_featurePoint;		  //归一化相机坐标
			Vector3d current_observation_feature; //像素坐标
			Vector2d current_featureVelocity;	 //归一化相机坐标下点的运动速度

			Eigen::Matrix3d K;
			K << 640.0, 0, 255,
				0, 640.0, 255,
				0, 0, 0;

			std::istringstream ssfeatureData(featuresImage_line);
			ssfeatureData >> current_featurePoint.x() >> current_featurePoint.y();
			featurePoint.push_back(current_featurePoint);
			feature_id.push_back(ids);

			current_featureVelocity.x() = (current_featurePoint.x() - lastfeaturePoint[ids].x()) / (dStampNSec - lastTime);
			current_featureVelocity.y() = (current_featurePoint.y() - lastfeaturePoint[ids].y()) / (dStampNSec - lastTime);
			featureVelocity.push_back(current_featureVelocity);

			current_observation_feature = Vector3d(current_featurePoint.x(), current_featurePoint.y(), 1);
			current_observation_feature = K * current_observation_feature;

			observation_feature.push_back(Vector2d(current_observation_feature.x(), current_observation_feature.y()));

			//可视化图像
			cv::circle(show_img, cv::Point2f(current_observation_feature.x(), current_observation_feature.y()), 2, cv::Scalar(255, 225, 255), 2);
			ids++;
		}
		featuresImage.close();
		lastTime = dStampNSec;
		lastfeaturePoint = featurePoint;
		pSystem->PubFeatureData(dStampNSec, feature_id, featurePoint, observation_feature, featureVelocity); //带时间戳的feature point数据载入系统，

		//是否可视化追踪过程
		if (1)
		{
			cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
			cv::imshow("IMAGE", show_img);
			cv::waitKey(1);
		}
		usleep(50000 * nDelayTimes);
	}
}

int main(int argc, char **argv)
{
	pSystem.reset(new System(sConfig_file));

	//启动多线程
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

	sleep(1);
	std::thread thd_PubImuData(PubImuData);		//imu数据的预处理－＞imu buf
	std::thread thd_PubImageData(PubImageData); //image数据预处理-> image buf

	std::thread thd_PubRealData(PubRealData);	 //imu数据的预处理－＞imu buf
	std::thread thd_Draw(&System::Draw, pSystem); //轨迹实时可视化的线程

	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;

	getchar();
	return 0;
}

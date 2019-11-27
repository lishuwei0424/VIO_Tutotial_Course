#include "estimator.h"

#include "backend/vertex_inverse_depth.h"
#include "backend/vertex_pose.h"
#include "backend/vertex_speedbias.h"
#include "backend/edge_reprojection.h"
#include "backend/edge_imu.h"

#include <ostream>
#include <fstream>

using namespace myslam;

Estimator::Estimator() : f_manager{Rs}
{
    // ROS_INFO("init begins");
    clearState();
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        // cout << "1 Estimator::setParameter tic: " << tic[i].transpose()
        //     << " ric: " << ric[i] << endl;
    }
    cout << "1 Estimator::setParameter FOCAL_LENGTH: " << FOCAL_LENGTH << endl;
    f_manager.setRic(ric);
    project_sqrt_info_ = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    //初始化相机和IMU的相对姿态
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    //
    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL; //初始化求解的阶段
    first_imu = false;     //第一帧标志位
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;

    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;

    tmp_pre_integration = nullptr;

    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = 0;
    relocalization_info = 0;

    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

/*******************************************************Estimator::processIMU****************************************************************
 @brief       处理IMU数据
 @Description IMU预积分，中值积分得到当前PQV作为优化初值
 @param[in]   dt 时间间隔
 @param[in]   linear_acceleration 线加速度
 @param[in]   angular_velocity 角速度
 @return      void
*********************************************************Estimator::processIMU***************************************************************/
void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);

        //if(solver_flag != NON_LINEAR)
        tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

/**************************************************processImage()***********************************************************
Estimator::processImage() 处理图像
1.  FeatureManager::addFeatureCheckParallax() 通过检测两帧之间的视差决定是否作为关键帧，同时添加之前检测到的特征点到feature（list< FeaturePerId >）这个容器中，
    计算每一个点跟踪的次数，以及它的视差
        FeatureManager::compensatedParallax2() 对相近的特征点进行视差计算
2.  是否初始化camera与IMU之间的外参
    FeatureManager::getCorresponding() 得到两帧之间特征点关系
    InitialEXRotation::CalibrationExRotation() 得到camera与IMU之间的旋转偏移常量
    Estimator::initialStructure() 视觉结构初始化
        通过重力协方差检测IMU的可观测性
        Estimator::relativePose() 在滑动窗口中选择两帧中有足够多特征点和视差的帧l，利用五点法恢复相对旋转和平移量
            MotionEstimator::solveRelativeRT() 利用五点法求解相机初始位姿
                cv::findFundamentalMat() 利用opencv函数求解F矩阵
                cv::recoverPose() 利用opencv函数分解F矩阵，得到相机旋转量、位移量
        GlobalSFM::construct() 全局SFM初始化全部初始帧中的相机位置和特征点空间3D位置（l是F矩阵初始化得到的，下面做的SFM，就是将l帧作为系统初始化原点，通过PnP和三角化得到滑动窗口中l帧之后和l帧之前的相机位置和3D特征点）
            1). 三角化l帧与frame_num-1帧 GlobalSFM::triangulatePoint()
            2). 对l+1, l+2, l+3, … frame_num-2 帧与sfm_f特征点队列进行PnP求解GlobalSFM::solveFrameByPnP()，得到这些帧的相机在空间的位置，三角化 l, l+1， l+2, … frame_num-2 帧与frame_num-1帧，得到这些特征点在空间的3D位置
            3). 三角化l+1, l+2 … frame_num-2帧与l帧
            4). 对l-1, l-2, l-3, …, 0帧与sfm_f特征点队列进行PnP求解，得到这些帧的相机在空间的位置,三角化l-1, l-2, l-3, …, 0帧与l帧
            5). 三角化其他所有的点
            6). SFM全局BA优化
        将相机坐标系转换到IMU坐标系中，然后再一次进行PnP求解，3D特征点还是使用之前SFM中求解出来的，后续也没有进行优化
        Estimator::visualInitialAlign() cameara与IMU对齐
            VisualIMUAlignment() 对齐camera与IMU (initial_aligmentc.cpp)
                VisualIMUAlignment() IMU陀螺仪零偏初始化；主要是通过滑动窗口中，每两帧之间通过SFM求解出来的旋转与IMU预积分的旋转量组成一个最小二乘法形式的等式，求解出来陀螺仪的零偏。
                LinearAlignment() 速度，重力向量，尺度初始化；由于在视觉初始化SFM的过程中，将其中位姿变化较大的两帧中使用E矩阵求解出来旋转和位移，后续的PnP和三角化都是在这个尺度下完成的。所以当前的尺度与IMU测量出来的真实世界尺度肯定不是一致的，所以需要这里进行对齐。这里对齐的方法主要是通过在滑动窗口中每两帧之间的位置和速度与IMU预积分出来的位置和速度组成一个最小二乘法的形式，然后求解出来
                    RefineGravity() 进一步细化重力加速度，提高估计值的精度，形式与LinearAlignment()是一致的，只是将g gg改为$g \cdot \mathbf{\bar{ \hat{g}}} + w_1 \mathbf b_1 + w_2 \mathbf b_2 $
            FeatureManager::triangulate() 三角化
            更新相机速度，位置和旋转量(通过精确求解的尺度，重力向量)
        Estimator::solveOdometry()
            FeatureManager::triangulate() 三角化
            Estimator::optimization()
                添加ceres的参数块，在滑动窗口中机体在世界坐标系的坐标，速度，旋转，IMU的陀螺仪零偏，加速度零偏。
                Estimator::vector2double() 系统数据类型转换，由Ps,Rs转换为para_Pose，Vs,Bas,Bgs转换为para_SpeedBias，tic,ric转换为para_Ex_Pose。
                添加先验残差，IMU测量残差，camera测量残差。注意这里IMU项和camera项之间是有一个系数，这个系数就是他们各自的协方差矩阵：IMU的协方差是预积分的协方差(IMUFactor::Evaluate，中添加IMU协方差，求解jacobian矩阵)，而camera的测量残差则是一个固定的系数（f/1.5 f/1.5f/1.5）
                如果进行闭环优化的话，还要添加闭环检测优化残差，计算滑动窗口中与每一个闭环关键帧的相对位姿，这个相对位置是为后面的图优化准备
            边缘化处理
                如果第二最新帧是关键帧
                    向ResidualBlockInfo容器中(factors)添加先验残差，最新IMU测量残差，camera所有特征点测量残差
                    MarginalizationInfo::preMarginalize()
                        根据各个测量模型Evaluate() 计算残差；各个参数块拷贝到统一的内存（parameter_block_data）中
                    MarginalizationInfo::marginalize()
                        利用多线程构造稀疏矩阵H
                        利用舒尔补消元简化稀疏矩阵求解过程
                        滑动窗口中参数块存储地址调整
                    如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉（边缘化）而保留IMU测量值在滑动窗口中。（其他步骤和上一步骤相同）
                    Estimator::double2vector() 系统数据类型转换，与vector2double()过程相反。通过闭环检测得到最新一个关键帧之前跟踪得到的位姿与闭环检测得到的位姿得到矫正量
                    Estimator::failureDetection() 检测SLAM系统是否失败
        Estimator::slideWindow() 滑动窗口all_image_frame，维持滑动窗口的大小，保证SLAM运行计算的复杂度。如果第二最新帧是关键帧的话，那么这个关键帧就会留在滑动窗口中，时间最长的一帧和其测量值就会被边缘化掉如果第二最新帧不是关键帧的话，则把这帧的视觉测量舍弃掉而保留IMU测量值在滑动窗口中这样的策略会保证系统的稀疏性。(0, 1, …, N)关键帧，0是时间最长的关键帧，N是最新关键帧。
            Estimator::slideWindowOld() marginalization_flag == MARGIN_OLD
                FeatureManager::removeBack() 将特征点进行处理，将每一个特征点开始出现的帧号减一，如果这个关键点的帧号为0，则直接删除
            Estimator::slideWindowNew() marginalization_flag == MARGIN_SECOND_NEW
                FeatureManager::removeFront()
 ***************************************************processImage()***********************************************************/
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double header)
{
    //ROS_DEBUG("new image coming ------------------------------------------");
    // cout << "Adding feature points: " << image.size()<<endl;
    //添加之前检测到的特征点到feature容器中，计算每一个点跟踪的次数，以及它的视差
    //通过检测两帧之间的视差决定次新帧是否作为关键帧
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;

    //ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    //ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    //ROS_DEBUG("Solving %d", frame_count);
    // cout << "number of feature: " << f_manager.getFeatureCount()<<endl;
    Headers[frame_count] = header;

    //将图像数据、时间、临时预积分值存到图像帧类中
    ImageFrame imageframe(image, header);
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header, imageframe));

    //更新临时预积分初始值
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if (ESTIMATE_EXTRINSIC == 2)
    {
        cout << "calibrating extrinsic param, rotation movement is needed" << endl;
        if (frame_count != 0)
        {
            //得到两帧之间归一化特征点
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);

            /************************************************************************************************************************
            VINS外参标定指的是对相机坐标系到IMU坐标系的变换矩阵进行在线标定与优化。
            在参数配置文件yaml中，参数estimate_extrinsic反映了外参的情况：
                1、等于0表示这外参已经是准确的了，之后不需要优化；
                2、等于1表示外参只是一个估计值，后续还需要将其作为初始值放入非线性优化中；
                3、等于2表示不知道外参，需要进行标定
            ***********************************************************************************************************************/
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                // ROS_WARN("initial extrinsic rotation calib success");
                // ROS_WARN_STREAM("initial extrinsic rotation: " << endl
                //    << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

    /************************************************************************************************************************
    总的来说，视觉惯性对齐主要包括以下流程：
    1、若旋转外参数qbc q未知，则先估计旋转外参
    2、利用旋转约束估计陀螺仪bias
    3、利用平移约束估计重力方向，速度，以及尺度初始值
    4、对重力向量gc0 g^{c_0}g 进一步优化
    5、求解世界坐标系w和初始相机坐标系c
    ************************************************************************************************************************/
    if (solver_flag == INITIAL)
    {
        //frame_count是滑动窗口中图像帧的数量，一开始初始化为0，滑动窗口总帧数WINDOW_SIZE=10
        //确保有足够的frame参与初始化
        if (frame_count == WINDOW_SIZE)
        {
            //初始化是否成功
            bool result = false;

            //有外参且当前帧时间戳大于初始化时间戳0.1秒，就进行初始化操作
            if (ESTIMATE_EXTRINSIC != 2 && (header - initial_timestamp) > 0.1)
            {
                // cout << "1 initialStructure" << endl;
                //视觉惯性联合初始化
                result = initialStructure();

                //更新初始化时间戳
                initial_timestamp = header;
            }
            if (result)
            {
                //先进行一次滑动窗口非线性优化，得到当前帧与第一帧的位姿
                solver_flag = NON_LINEAR;
                solveOdometry();
                slideWindow();
                f_manager.removeFailures();
                cout << "Initialization finish!" << endl;
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
            }
            else
                //初始化失败则直接滑动窗口
                slideWindow();
        }
        else
            //图像帧数量+1
            frame_count++;
    }
    else //紧耦合的非线性优化
    {
        TicToc t_solve;
        solveOdometry();
        //ROS_DEBUG("solver costs: %fms", t_solve.toc());

        //故障检测与恢复,一旦检测到故障，系统将切换回初始化阶段
        if (failureDetection())
        {
            // ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            // ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow();
        f_manager.removeFailures();

        //ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // prepare output of VINS
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

/*************************************************************************************************************************
 * @brief       视觉的结构初始化
 * @Description 确保IMU有充分运动激励
 *              relativePose()找到具有足够视差的两帧,由F矩阵恢复R、t作为初始值
 *              sfm.construct() 全局纯视觉SFM 恢复滑动窗口帧的位姿
 *              visualInitialAlign()视觉惯性联合初始化
 * @return      bool true:初始化成功
************************************************************************************************************************/
bool Estimator::initialStructure()
{
    TicToc t_sfm;

    //1、通过计算线加速度的标准差，判断IMU是否有充分运动激励，以进行初始化
    //check imu observibility
    {
        //注意这里并没有算上all_image_frame的第一帧，所以求均值和标准差的时候要减一
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }

        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);

        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if (var < 0.25)
        {
            // ROS_INFO("IMU excitation not enouth!");
            //return false;
        }
    }

    //2.将f_manager中的所有feature保存到vector<SFMFeature> sfm_f
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    }
    
    //3.保证具有足够的视差,由E矩阵恢复R、t这里的第L帧是从第一帧开始到滑动窗口中第一个满足与当前帧的平均视差足够大的帧，会作为参考帧到下面的全局sfm使用，得到的Rt为当前帧到第l帧的坐标系变换Rt
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        cout << "Not enough features or parallax; Move device around" << endl;
        return false;
    }

    //4、对窗口中每个图像帧求解sfm问题，得到所有图像帧相对于参考帧的旋转四元数Q、平移向量T和特征点坐标sfm_tracked_points。
    GlobalSFM sfm;
    if (!sfm.construct(frame_count + 1, Q, T, l,
                       relative_R, relative_T,
                       sfm_f, sfm_tracked_points))
    {
        //求解失败则边缘化最早一帧并滑动窗口
        cout << "global SFM failed!" << endl;
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //对于所有的图像帧，包括不在滑动窗口中的，提供初始的RT估计，然后solvePnP进行优化,得到每一帧的姿态
    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin();
    for (int i = 0; frame_it != all_image_frame.end(); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        if ((frame_it->first) == Headers[i])
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if ((frame_it->first) > Headers[i])
        {
            i++;
        }

        //Q和T是图像帧的位姿，而不是求解PNP时所用的坐标系变换矩阵
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = -R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);

        //罗德里格斯公式将旋转矩阵转换成旋转向量
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if (it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
        if (pts_3_vector.size() < 6)
        {
            cout << "Not enough points for solve pnp pts_3_vector size " << pts_3_vector.size() << endl;
            return false;
        }

        /************************************************************************************************************************** 
        *bool cv::solvePnP(    求解pnp问题
        *   InputArray  objectPoints,   特征点的3D坐标数组
        *   InputArray  imagePoints,    特征点对应的图像坐标
        *   InputArray  cameraMatrix,   相机内参矩阵
        *   InputArray  distCoeffs,     失真系数的输入向量
        *   OutputArray     rvec,       旋转向量
        *   OutputArray     tvec,       平移向量
        *   bool    useExtrinsicGuess = false, 为真则使用提供的初始估计值
        *   int     flags = SOLVEPNP_ITERATIVE 采用LM优化
        *)   
        *************************************************************************************************************************/
        if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            cout << " solve pnp fail!" << endl;
            return false;
        }

        //这里也同样需要将坐标变换矩阵转变成图像帧位姿，并转换为IMU坐标系的位姿
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp, tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }

    if (visualInitialAlign())
        return true;
    else
    {
        cout << "misalign visual structure with IMU" << endl;
        return false;
    }
}

/************************************************************************************************************************** 
bool Estimator::visualInitialAlign() 视觉惯性联合初始化:
该函数主要实现了陀螺仪的偏置校准(加速度偏置没有处理)，计算速度V[0:n]、重力g、尺度s。
同时更新了Bgs后，IMU测量量需要repropagate；得到尺度s和重力g的方向后，需更新所有图像帧在世界坐标系下的Ps、Rs、Vs
**************************************************************************************************************************/
bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;

    //1.1、计算陀螺仪偏置，尺度，重力加速度和速度
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if (!result)
    {
        //ROS_DEBUG("solve g failed!");
        return false;
    }

    //2、获取所有图像帧的位姿Ps、Rs，并将其置为关键帧
    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i]].R;
        Vector3d Pi = all_image_frame[Headers[i]].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i]].is_key_frame = true;
    }

    //将所有特征点的深度置为-1
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //重新计算特征点的深度
    //triangulat on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for (int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    //4、陀螺仪的偏置bgs改变，重新计算预积分
    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }

    //5、将Ps、Vs、depth尺度s缩放后转变为相对于第0帧图像坐标系下,论文提到的以第一帧c0为基准坐标系，通过相机坐标系ck位姿得到IMU坐标系bk位姿,
    //之前都是以第l帧为基准坐标系，转换到以第一帧b0为基准坐标系的话应该是：
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);

    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if (frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    //6、通过将重力旋转到z轴上，得到世界坐标系与摄像机坐标系c0之间的旋转矩阵rot_diff
    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;

    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }

    //ROS_DEBUG_STREAM("g0     " << g.transpose());
    //ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

//该函数判断每帧到窗口最后一帧对应特征点的平均视差大于30，且内点数目大于12则可进行初始化，同时返回当前帧到第l帧的坐标系变换R和T
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if (average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                //ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        //cout << "triangulation costs : " << t_tri.toc() << endl;
        backendOptimization();
    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

void Estimator::double2vector()
{
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                     para_Pose[0][3],
                                                     para_Pose[0][4],
                                                     para_Pose[0][5])
                                             .toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0));
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        //ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5])
                               .toRotationMatrix()
                               .transpose();
    }

    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();

        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                    para_Pose[i][1] - para_Pose[0][1],
                                    para_Pose[i][2] - para_Pose[0][2]) +
                origin_P0;

        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5])
                     .toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // relative info between two loop frame
    if (relocalization_info)
    {
        Matrix3d relo_r;
        Vector3d relo_t;
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) +
                 origin_P0;
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        //cout << "vins relo " << endl;
        //cout << "vins relative_t " << relo_relative_t.transpose() << endl;
        //cout << "vins relative_yaw " <<relo_relative_yaw << endl;
        relocalization_info = 0;
    }
}

bool Estimator::failureDetection()
{
    //在最新帧中跟踪的特征数小于某一阈值
    if (f_manager.last_track_num < 2)
    {
        //ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }

    //偏置或外部参数估计有较大的变化
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        //ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        //ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    /*
    if (tic(0) > 1)
    {
        //ROS_INFO(" big extri param estimation %d", tic(0) > 1);
        return true;
    }
    */

    //最近两个估计器输出之间的位置或旋转有较大的不连续性
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        //ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        //ROS_INFO(" big z translation");
        return true;
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        //ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::MargOldFrame()
{
    backend::LossFunction *lossfunction;
    lossfunction = new backend::CauchyLoss(1.0);

    // step1. 构建 problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    int pose_dim = 0;

    // 先把 外参数 节点加入图优化，这个节点在以后一直会被用到，所以我们把他放在第一个
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {
        Eigen::VectorXd pose(7);
        pose << para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5], para_Ex_Pose[0][6];
        vertexExt->SetParameters(pose);
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Eigen::VectorXd pose(7);
        pose << para_Pose[i][0], para_Pose[i][1], para_Pose[i][2], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5], para_Pose[i][6];
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();
      
        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2],
            para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5],
            para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }

    // IMU 
    {   
        if (pre_integrations[1]->sum_dt < 10.0)
        {
            std::shared_ptr<backend::EdgeImu> imuEdge(new backend::EdgeImu(pre_integrations[1]));
            std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
            edge_vertex.push_back(vertexCams_vec[0]);
            edge_vertex.push_back(vertexVB_vec[0]);
            edge_vertex.push_back(vertexCams_vec[1]);
            edge_vertex.push_back(vertexVB_vec[1]);
            imuEdge->SetVertex(edge_vertex);
            problem.AddEdge(imuEdge);
        }
    }

    // Visual Factor
    {
        int feature_index = -1;
        // 遍历每一个特征
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            ++feature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            if (imu_i != 0)
                continue;

            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            shared_ptr<backend::VertexInverseDepth> verterxPoint(new backend::VertexInverseDepth());
            VecX inv_d(1);

            inv_d << para_Feature[feature_index][0];
            verterxPoint->SetParameters(inv_d);
            problem.AddVertex(verterxPoint);

            // 遍历所有的观测
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {   
                imu_j++;
                if (imu_i == imu_j)
                    continue;

                Vector3d pts_j = it_per_frame.point;
          
                std::shared_ptr<backend::EdgeReprojection> edge(new backend::EdgeReprojection(pts_i, pts_j));
                std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
                edge_vertex.push_back(verterxPoint);
                edge_vertex.push_back(vertexCams_vec[imu_i]);
                edge_vertex.push_back(vertexCams_vec[imu_j]);
                edge_vertex.push_back(vertexExt);

                edge->SetVertex(edge_vertex);
                edge->SetInformation(project_sqrt_info_.transpose() * project_sqrt_info_);

                edge->SetLossFunction(lossfunction);
                problem.AddEdge(edge);
            }
        }
    }

    // 先验
    {
        // 已经有 Prior 了
        if (Hprior_.rows() > 0)
        {
            problem.SetHessianPrior(Hprior_); // 告诉这个 problem
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);
            problem.ExtendHessiansPriorSize(15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
        }
        else
        {
            Hprior_ = MatXX(pose_dim, pose_dim);
            Hprior_.setZero();
            bprior_ = VecX(pose_dim);
            bprior_.setZero();
            problem.SetHessianPrior(Hprior_); // 告诉这个 problem
            problem.SetbPrior(bprior_);
        }
    }

    std::vector<std::shared_ptr<backend::Vertex>> marg_vertex;
    marg_vertex.push_back(vertexCams_vec[0]);
    marg_vertex.push_back(vertexVB_vec[0]);
    problem.Marginalize(marg_vertex, pose_dim);
    Hprior_ = problem.GetHessianPrior();
    bprior_ = problem.GetbPrior();
    errprior_ = problem.GetErrPrior();
    Jprior_inv_ = problem.GetJtPrior();
}

void Estimator::MargNewFrame()
{
    // step1. 构建 problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    //    vector<backend::Point3d> points;
    int pose_dim = 0;

    // 先把 外参数 节点加入图优化，这个节点在以后一直会被用到，所以我们把他放在第一个
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {   
        Eigen::VectorXd pose(7);
        pose << para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5], para_Ex_Pose[0][6];
        vertexExt->SetParameters(pose);
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Eigen::VectorXd pose(7);
        pose << para_Pose[i][0], para_Pose[i][1], para_Pose[i][2], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5], para_Pose[i][6];
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();

        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2],
            para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5],
            para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }

    // 先验
    {
        // 已经有 Prior 了
        if (Hprior_.rows() > 0)
        {
            problem.SetHessianPrior(Hprior_); // 告诉这个 problem
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);

            problem.ExtendHessiansPriorSize(15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
        }
        else
        {
            Hprior_ = MatXX(pose_dim, pose_dim);
            Hprior_.setZero();
            bprior_ = VecX(pose_dim);
            bprior_.setZero();
        }
    }

    std::vector<std::shared_ptr<backend::Vertex>> marg_vertex;
    // 把窗口倒数第二个帧 marg 掉
    marg_vertex.push_back(vertexCams_vec[WINDOW_SIZE - 1]);
    marg_vertex.push_back(vertexVB_vec[WINDOW_SIZE - 1]);
    problem.Marginalize(marg_vertex, pose_dim);
    Hprior_ = problem.GetHessianPrior();
    bprior_ = problem.GetbPrior();
    errprior_ = problem.GetErrPrior();
    Jprior_inv_ = problem.GetJtPrior();
}

void Estimator::problemSolve()
{
    backend::LossFunction *lossfunction;
    lossfunction = new backend::CauchyLoss(1.0);
    //lossfunction = new backend::TukeyLoss(1.0);

    // step1. 构建 problem
    backend::Problem problem(backend::Problem::ProblemType::SLAM_PROBLEM);
    vector<shared_ptr<backend::VertexPose>> vertexCams_vec;
    vector<shared_ptr<backend::VertexSpeedBias>> vertexVB_vec;
    int pose_dim = 0;
    
    // 先把 外参数 节点加入图优化，这个节点在以后一直会被用到，所以我们把他放在第一个
    shared_ptr<backend::VertexPose> vertexExt(new backend::VertexPose());
    {   
        Eigen::VectorXd pose(7);
        pose << para_Ex_Pose[0][0], para_Ex_Pose[0][1], para_Ex_Pose[0][2], para_Ex_Pose[0][3], para_Ex_Pose[0][4], para_Ex_Pose[0][5], para_Ex_Pose[0][6];
        vertexExt->SetParameters(pose);

        if (!ESTIMATE_EXTRINSIC)
        {
            //ROS_DEBUG("fix extinsic param");
            // TODO:: set Hessian prior to zero
            vertexExt->SetFixed();
        }
        else
        {
            //ROS_DEBUG("estimate extinsic param");
        }
        problem.AddVertex(vertexExt);
        pose_dim += vertexExt->LocalDimension();
    }

    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {   
        shared_ptr<backend::VertexPose> vertexCam(new backend::VertexPose());
        Eigen::VectorXd pose(7);
        pose << para_Pose[i][0], para_Pose[i][1], para_Pose[i][2], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5], para_Pose[i][6];
        vertexCam->SetParameters(pose);
        vertexCams_vec.push_back(vertexCam);
        problem.AddVertex(vertexCam);
        pose_dim += vertexCam->LocalDimension();
      
        shared_ptr<backend::VertexSpeedBias> vertexVB(new backend::VertexSpeedBias());
        Eigen::VectorXd vb(9);
        vb << para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2],
            para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5],
            para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8];
        vertexVB->SetParameters(vb);
        vertexVB_vec.push_back(vertexVB);
        problem.AddVertex(vertexVB);
        pose_dim += vertexVB->LocalDimension();
    }

    // IMU
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        
        std::shared_ptr<backend::EdgeImu> imuEdge(new backend::EdgeImu(pre_integrations[j]));
        std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
        edge_vertex.push_back(vertexCams_vec[i]);
        edge_vertex.push_back(vertexVB_vec[i]);
        edge_vertex.push_back(vertexCams_vec[j]);
        edge_vertex.push_back(vertexVB_vec[j]);
        imuEdge->SetVertex(edge_vertex);
        problem.AddEdge(imuEdge);
    }

    // Visual Factor
    vector<shared_ptr<backend::VertexInverseDepth>> vertexPt_vec;
    {
        int feature_index = -1;
        // 遍历每一个特征
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;

            ++feature_index;

            int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point;

            shared_ptr<backend::VertexInverseDepth> verterxPoint(new backend::VertexInverseDepth());
            VecX inv_d(1);
            inv_d << para_Feature[feature_index][0];
            verterxPoint->SetParameters(inv_d);
            problem.AddVertex(verterxPoint);
            vertexPt_vec.push_back(verterxPoint);

            // 遍历所有的观测
            for (auto &it_per_frame : it_per_id.feature_per_frame)
            {
                imu_j++;
                if (imu_i == imu_j)
                    continue;

                Vector3d pts_j = it_per_frame.point;

                std::shared_ptr<backend::EdgeReprojection> edge(new backend::EdgeReprojection(pts_i, pts_j));
                std::vector<std::shared_ptr<backend::Vertex>> edge_vertex;
                edge_vertex.push_back(verterxPoint);
                edge_vertex.push_back(vertexCams_vec[imu_i]);
                edge_vertex.push_back(vertexCams_vec[imu_j]);
                edge_vertex.push_back(vertexExt);

                edge->SetVertex(edge_vertex);
                edge->SetInformation(project_sqrt_info_.transpose() * project_sqrt_info_);

                edge->SetLossFunction(lossfunction);
                problem.AddEdge(edge);
            }
        }
    }

    // 先验
    {
        // 已经有 Prior 了
        if (Hprior_.rows() > 0)
        {
            // 外参数先验设置为 0. TODO:: 这个应该放到 solver 里去弄
            //            Hprior_.block(0,0,6,Hprior_.cols()).setZero();
            //            Hprior_.block(0,0,Hprior_.rows(),6).setZero();

            problem.SetHessianPrior(Hprior_); // 告诉这个 problem
            problem.SetbPrior(bprior_);
            problem.SetErrPrior(errprior_);
            problem.SetJtPrior(Jprior_inv_);
            problem.ExtendHessiansPriorSize(15); // 但是这个 prior 还是之前的维度，需要扩展下装新的pose
        }
    }
    
    problem.Solve(10);

    // update bprior_,  Hprior_ do not need update
    if (Hprior_.rows() > 0)
    {
        std::cout << "----------- update bprior -------------\n";
        std::cout << "             before: " << bprior_.norm() << std::endl;
        std::cout << "                     " << errprior_.norm() << std::endl;
        bprior_ = problem.GetbPrior();
        errprior_ = problem.GetErrPrior();
        std::cout << "             after: " << bprior_.norm() << std::endl;
        std::cout << "                    " << errprior_.norm() << std::endl;
    }

    // update parameter
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        VecX p = vertexCams_vec[i]->Parameters();
        for (int j = 0; j < 7; ++j)
        {
            para_Pose[i][j] = p[j];
        }

        VecX vb = vertexVB_vec[i]->Parameters();
        for (int j = 0; j < 9; ++j)
        {
            para_SpeedBias[i][j] = vb[j];
        }
    }

    // 遍历每一个特征
    for (int i = 0; i < vertexPt_vec.size(); ++i)
    {
        VecX f = vertexPt_vec[i]->Parameters();
        para_Feature[i][0] = f[0];
    }
}

/**************************************************************************************************************************** 
* @brief       基于滑动窗口紧耦合的非线性优化，残差项的构造和求解
* @Description 添加要优化的变量 (p,v,q,ba,bg) 一共15个自由度，IMU的外参也可以加进来
*              添加残差，残差项分为4块 先验残差+IMU残差+视觉残差+闭环检测残差
*              根据倒数第二帧是不是关键帧确定边缘化的结果           
* @return      void
***************************************************************************************************************************/
void Estimator::backendOptimization()
{
    TicToc t_solver;

    // 借助 vins 框架，维护变量
    vector2double();

    // 构建求解器
    problemSolve();

    // 优化后的变量处理下自由度
    double2vector();
    //ROS_INFO("whole time for solver: %f", t_solver.toc());
    
    // 维护 marg
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        vector2double();
        
        MargOldFrame();
        
        std::unordered_map<long, double *> addr_shift; // prior 中对应的保留下来的参数地址
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
    }
    else
    {
        if (Hprior_.rows() > 0)
        {

            vector2double();

            MargNewFrame();

            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
        }
    }
}

void Estimator::slideWindow()
{
    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0];
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;

                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);
            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

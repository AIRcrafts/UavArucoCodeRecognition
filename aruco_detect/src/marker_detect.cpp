#include <time.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <numeric>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// ros 头文件
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// opencv 头文件
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

// topic 头文件
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>

#include "aruco_detect/DetectionInfo.h"

using namespace std;

// 【接收】输入图像
image_transport::Subscriber image_sub;
// 【发布】识别后的图像
image_transport::Publisher image_detected_pub;
// 【发布】目标在相机下的位姿
ros::Publisher position_pub;

// 图像相关参数
cv::Mat img;
aruco_detect::DetectionInfo pose_info;
// 相机话题中的图像同步相关变量
int frame_width, frame_height;
cv::Mat cam_image_copy;
boost::shared_mutex mutex_image_callback;   // 并发锁
bool image_status = false;
boost::shared_mutex mutex_image_status;

// 接收摄像头传入图像，并将图像保存在 cam_image_copy 中
void cameraCB(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cam_image;
    try
    {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        cerr << e.what() << '\n';
        return;
    }

    if (cam_image)
    {
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            cam_image_copy = cam_image->image.clone();
        }
        {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutex_image_status);
            image_status = true;
        }
        frame_width = cam_image->image.size().width;
        frame_height = cam_image->image.size().height;
    }
    
    return;
}

// 用此函数查看是否收到图像话题
bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutex_image_status);
    return image_status;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"aruco_detect_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    // 接收launch文件输入参数
    string camera_topic_, camera_info_;

    if (nh.getParam("camera_topic", camera_topic_))
    {
        ROS_INFO("camera topic is %s", camera_topic_.c_str());
    }
    else
    {
        ROS_ERROR("not received camear topic");
    }

    if (nh.getParam("camera_info", camera_info_))
    {
        ROS_INFO("received camera info ");
    }
    else
    {
        ROS_ERROR("not received camear topic");
    }

    // 【发布】
    position_pub = nh.advertise<aruco_detect::DetectionInfo>("/uav/aruco_detection", 10);
    // 【接收】
    image_sub = it.subscribe(camera_topic_, 1, cameraCB);
    // 【发布】检测结果
    image_detected_pub = it.advertise("/image_detected", 1);

    sensor_msgs::ImagePtr msg_ellipse;

    // 读取参数文档 camera_param.yaml 中的参数值
    YAML::Node camera_config = YAML::LoadFile(camera_info_);
    // 相机内部参数
    double fx = camera_config["fx"].as<double>();
    double fy = camera_config["fy"].as<double>();
    double cx = camera_config["x0"].as<double>();
    double cy = camera_config["y0"].as<double>();
    // 相机畸变参数
    double k1 = camera_config["k1"].as<double>();
    double k2 = camera_config["k2"].as<double>();
    double p1 = camera_config["p1"].as<double>();
    double p2 = camera_config["p2"].as<double>();
    double k3 = camera_config["k3"].as<double>();

    double landpad_det_len = camera_config["landpad_det_len"].as<double>();

    // 相机参数赋值
    // 相机内参
    cv::Mat camera_matrix;
    camera_matrix = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    camera_matrix.ptr<double>(0)[0] = fx;
    camera_matrix.ptr<double>(0)[2] = cx;
    camera_matrix.ptr<double>(1)[1] = fy;
    camera_matrix.ptr<double>(1)[2] = cy;
    camera_matrix.ptr<double>(2)[2] = 1.0f;
    // 相机畸变参数k1 k2 p1 p2 k3
    cv::Mat distortion_coefficients;
    distortion_coefficients = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
    distortion_coefficients.ptr<double>(0)[0] = k1;
    distortion_coefficients.ptr<double>(1)[0] = k2;
    distortion_coefficients.ptr<double>(2)[0] = p1;
    distortion_coefficients.ptr<double>(3)[0] = p2;
    distortion_coefficients.ptr<double>(4)[0] = k3;

    // ArUco Marker字典选择
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // 旋转向量和转移向量初始化
    vector<double> rv(3), tv(3);
    cv::Mat rvec(rv), tvec(tv);

    // 上一时刻的位置、yaw、加速度和四元素等参数
    float last_x(0), last_y(0), last_z(0), last_yaw(0), last_az(0), last_ay(0), last_ax(0), last_qx(0), last_qy(0), last_qz(0), last_qw(0);

    ros::Rate rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        if (!getImageStatus() && ros::ok())
        {
            continue;
        }

        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutex_image_callback);
            img = cam_image_copy.clone();
        }

        // ArUco 检测
        // markerids 存储识别到的二维码的编号 markerCorners 每个二维码对应的四个角点的像素坐标 (此次实验仅识别一个aruco码)
        vector<int> markerid, markerid_deted;  
        vector<vector<cv::Point2f>> markerCorners, markerCorners_deted, rejectedCandidate;

        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::aruco::detectMarkers(img, dictionary, markerCorners_deted, markerid_deted, parameters, rejectedCandidate);

        int pre_marker_id = 0;  // 此实验识别 aruco码id为0
        for (int i = 0; i < markerid_deted.size(); i++)
        {
            if (markerid_deted[i] == pre_marker_id)
            {
                markerid.push_back(pre_marker_id);
                markerCorners.push_back(markerCorners_deted[i]);
                ROS_INFO("id0 aruco detected!");
                break;
            }
            
        }

        // 不同id有着不同的大小参数
        if (!markerid.empty())
        {
            // 可视化
            cv::aruco::drawDetectedMarkers(img, markerCorners, markerid);
            double center_point[2];
            // 默认实验中仅会识别到一个aruco码，所以取第一组角点坐标取平均值，得到二维码原点
            center_point[0] = (markerCorners[0][0].x + markerCorners[0][1].x + markerCorners[0][2].x + markerCorners[0][3].x) * 0.25;
            center_point[1] = (markerCorners[0][0].y + markerCorners[0][1].y + markerCorners[0][2].y + markerCorners[0][3].y) * 0.25;
            // 赋值消息参数
            pose_info.pixel_position[0] = (int)center_point[0];
            pose_info.pixel_position[1] = (int)center_point[1];

            vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, landpad_det_len, camera_matrix, distortion_coefficients, rvecs, tvecs);
            cv::aruco::drawAxis(img, camera_matrix, distortion_coefficients, rvecs[0], tvecs[0], landpad_det_len * 0.5f);

            // 姿态
            cv::Mat rotation_matrix;
            // 相机姿态：旋转向量 -> 旋转矩阵
            cv::Rodrigues(rvecs[0], rotation_matrix);
            Eigen::Matrix3d rotation_matrix_eigen;
            cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);
            Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix_eigen);
            q.normalize();  // 标准化

            // 位置
            std::vector<double> vec_t{tvecs[0][0], tvecs[0][1], tvecs[0][2]};
            cv::Mat vec_t_mat{vec_t};
            vec_t_mat = vec_t_mat;
            vec_t_mat.convertTo(vec_t_mat, CV_32FC1);

            // 原点位置
            double id_to8_t[3]={0,0,0};
            cv::Mat id_to8_t_mat = cv::Mat(3, 1, CV_32FC1, id_to8_t);
            rotation_matrix.convertTo(rotation_matrix, CV_32FC1);
            cv::Mat id_8_t = rotation_matrix * id_to8_t_mat + vec_t_mat;

            // 整合数据发布
            float o_tx = id_8_t.at<float>(0);
            float o_ty = id_8_t.at<float>(1);
            float o_tz = id_8_t.at<float>(2);

            // 将解算的位置转化成旋转矩阵 并旋转计算无人机相对于目标的位置
            float r11 = rotation_matrix.ptr<float>(0)[0];
            float r12 = rotation_matrix.ptr<float>(0)[1];
            float r13 = rotation_matrix.ptr<float>(0)[2];
            float r21 = rotation_matrix.ptr<float>(1)[0];
            float r22 = rotation_matrix.ptr<float>(1)[1];
            float r23 = rotation_matrix.ptr<float>(1)[2];
            float r31 = rotation_matrix.ptr<float>(2)[0];
            float r32 = rotation_matrix.ptr<float>(2)[1];
            float r33 = rotation_matrix.ptr<float>(2)[2];

            // 计算欧拉角; 旋转矩阵转换为欧拉角
            float thetaz = atan2(r21, r11) / CV_PI * 180;
            float thetay = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / CV_PI * 180;
            float thetax = atan2(r32, r33) / CV_PI * 180;

            // C2W代表 相机坐标系转换到世界坐标系姿态变化   W2C代表 世界坐标系转换到相机坐标系 Theta为欧拉角
            cv::Point3f Theta_C2W;      // 相机 -> 世界
            cv::Point3f Theta_W2C;      // 世界 -> 相机
            cv::Point3f Position_OcInW; // 相机坐标系下的位置

            Theta_C2W.z = thetaz;
            Theta_C2W.y = thetay;
            Theta_C2W.x = thetax;

            Theta_W2C.x = -1 * thetax;
            Theta_W2C.y = -1 * thetay;
            Theta_W2C.z = -1 * thetaz;

            Eigen::Vector3d eulerVec;
            eulerVec(0) = (Theta_C2W.z) / 180 * CV_PI;
            double A1_yaw = eulerVec(0);

            // 将解算后的位置发给控制端
            pose_info.header.stamp = ros::Time::now();
            pose_info.detected = true;
            pose_info.frame = 0;
            // 目标点位置
            pose_info.position[0] = o_tx;
            pose_info.position[1] = o_ty;
            pose_info.position[2] = o_tz;
            // ROS_INFO("x = %f, y = %f", pose_info.position[0], pose_info.position[1]);
            pose_info.attitude[0] = thetaz;
            pose_info.attitude[1] = thetay;
            pose_info.attitude[2] = thetax;
            pose_info.attitude_q[0] = q.x();
            pose_info.attitude_q[1] = q.y();
            pose_info.attitude_q[2] = q.z();
            pose_info.attitude_q[3] = q.w();
            // 视线角，球体坐标系
            pose_info.sight_angle[0] = atan(o_tx / o_tz);
            pose_info.sight_angle[1] = atan(o_ty / o_tz);
            // 偏航角误差
            pose_info.yaw_error = A1_yaw;

            // 记录历史值
            last_x = pose_info.position[0];
            last_y = pose_info.position[1];
            last_z = pose_info.position[2];
            last_az = pose_info.attitude[0];
            last_ay = pose_info.attitude[1];
            last_ax = pose_info.attitude[2];
            last_qx = pose_info.attitude_q[0];
            last_qy = pose_info.attitude_q[1];
            last_qz = pose_info.attitude_q[2];
            last_qw = pose_info.attitude_q[3];
            last_yaw = pose_info.yaw_error;

        }
        // 找不到就使用历史值
        // else
        // {
        //     pose_info.header.stamp = ros::Time::now();
        //     pose_info.detected = false;
        //     pose_info.frame = 0;
        //     pose_info.position[0] = last_x;
        //     pose_info.position[1] = last_y;
        //     pose_info.position[2] = last_z;
        //     pose_info.attitude[0] = last_az;
        //     pose_info.attitude[1] = last_ay;
        //     pose_info.attitude[2] = last_ax;
        //     pose_info.attitude_q[0] = last_qx;
        //     pose_info.attitude_q[1] = last_qy;
        //     pose_info.attitude_q[2] = last_qz;
        //     pose_info.attitude_q[3] = last_qw;
        //     pose_info.sight_angle[0] = atan(last_x / last_z);
        //     pose_info.sight_angle[1] = atan(last_y / last_z);
        //     pose_info.yaw_error = last_yaw;
        // }
        position_pub.publish(pose_info);

        msg_ellipse = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        image_detected_pub.publish(msg_ellipse);


        rate.sleep();
    }
    
    
    
    return 0;
}

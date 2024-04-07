#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include "eigen3/Eigen/Eigen"
#include "aruco_detect/DetectionInfo.h"
using namespace std;

vector<double> x_waypoints_list;    // 航点 x 坐标
vector<double> y_waypoints_list;    // 航点 y 坐标
int count_points = 0;               // 当前航点索引
int num_waypoints;                  // 总航点数
double desire_height_;              // 期望高度
geometry_msgs::PoseStamped aruco_pos;   // 检测到的二维码在世界坐标下位置
double deta_x, deta_y;              // 二维码相在无人机悬停时相对位置差
double aruco_pose_x = 0.9;
double aruco_pose_y = -1.9;
double att_w = 1;
double att_x = 0;
double att_y = 0;
double att_z = 0;

aruco_detect::DetectionInfo aruco_detection;
bool is_detect = false;
int pub_aruco = 0;
int count_hover = 0;
double temp_hover_x, temp_hover_y;

enum FlyState
{
    HOVER,      // 悬停
    RETURN,     // 返航
    LANDING,    // 降落
};

FlyState detected_flystate = HOVER;

// 接收当前状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// 接收飞机当前位置
geometry_msgs::PoseStamped pos_drone;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos_drone = *msg;
}

// 接收yaml文件传入的航点
void get_waypoints(ros::NodeHandle nh_)
{
    XmlRpc::XmlRpcValue waypoints;
    nh_.getParam("waypoints_list", waypoints);
    num_waypoints = waypoints.size();
    for (int i = 0; i < waypoints.size(); i++)
    {
        if (waypoints[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
            waypoints[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
            x_waypoints_list.push_back(double(waypoints[i][0]));
            y_waypoints_list.push_back(double(waypoints[i][1]));
        }   
    }

    ROS_INFO("The waypoints received are as follows");
    for (int ii = 0; ii < x_waypoints_list.size(); ii++)
    {
        ROS_INFO("x = %f, y = %f", x_waypoints_list[ii], y_waypoints_list[ii]);
    }
    ROS_INFO("-------------------------------------");
}

void detectCB(const aruco_detect::DetectionInfo::ConstPtr &msg)
{
    aruco_detection = *msg;
    is_detect = aruco_detection.detected;
    // cout << is_detect << endl;
    deta_x = aruco_detection.position[0];
    deta_y = aruco_detection.position[1];
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");

    // 【接收】
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    // 【接收】是否检测到aruco
    ros::Subscriber aruco_sub = nh.subscribe<aruco_detect::DetectionInfo>("/uav/aruco_detection", 10, detectCB);
    // 【发布】
    ros::Publisher aruco_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/aruco_pose", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    // 【客户端】
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 接收参数服务器中设置的参数
    get_waypoints(nh);
    nh.param("desire_height", desire_height_, 1.0);

    ros::Rate rate(20.0);
    ros::Rate rate_detected(20.0);

    // 等待与飞控连接
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = desire_height_;
    pose.pose.orientation.w = 0.707;
    pose.pose.orientation.z = 0.707;

    // 准备工作
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // 解锁并起飞
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (current_state.mode == "OFFBOARD" && current_state.armed &&
            abs(pos_drone.pose.position.x - x_waypoints_list[0]) < 0.1 &&
            abs(pos_drone.pose.position.y - y_waypoints_list[0]) < 0.1 &&
            abs(pos_drone.pose.position.z - desire_height_) < 0.1 )
        {
            ROS_INFO("sucess armed and offboard");
            break;
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    // 巡航
    while (ros::ok())
    {
        ros::spinOnce();

        if (is_detect)  // 检测到后：悬停5s，飞回起飞点，降落
        {
            if (count_hover == 0)
            {
                last_request = ros::Time::now();
                temp_hover_x = pos_drone.pose.position.x;
                temp_hover_y = pos_drone.pose.position.y;
                ROS_INFO("hover x=%f y=%f", temp_hover_x, temp_hover_y);
            }
            
            // cout << detected_flystate << endl;
            switch (detected_flystate)
            {
            case HOVER:
            {
                // ROS_INFO("hovering");
                if (ros::Time::now() - last_request <= ros::Duration(5.0))
                {
                    pose.pose.position.x = temp_hover_x;
                    pose.pose.position.y = temp_hover_y;
                    pose.pose.position.z = desire_height_;
                    // ROS_INFO("pose: %f, %f", pose.pose.position.x, pose.pose.position.y);
                    local_pos_pub.publish(pose);
                    count_hover += 1;

                }
                else
                {
                    detected_flystate = RETURN;
                }

                if (pub_aruco <= 5)
                {
                    aruco_pos.pose.position.x = aruco_pose_x; 
                    aruco_pos.pose.position.y = aruco_pose_y;
                    aruco_pos.pose.orientation.w = att_w;
                    aruco_pos.pose.orientation.x = att_x;
                    aruco_pos.pose.orientation.y = att_y;
                    aruco_pos.pose.orientation.z = att_z;
                    ROS_INFO("pub_aruco = %d aruco x = %f, y = %f",pub_aruco, aruco_pose_x, aruco_pose_y);
                    aruco_pos_pub.publish(aruco_pos);
                    pub_aruco += 1;
                }

                // if (ros::Time::now() - last_request > ros::Duration(3.0))
                // {
                //    // 发送二维码位置
                //     aruco_pos.pose.position.x = pos_drone.pose.position.x - deta_x;
                //     aruco_pos.pose.position.y = pos_drone.pose.position.y - deta_y;
                //     ROS_INFO("deta_x = %f, deta_y = %f", deta_x, deta_y);
                //     aruco_pos_pub.publish(aruco_pos);
                // }
                
            }
                break;
            case RETURN:
            {
                // ROS_INFO("returning");
                if (abs(pos_drone.pose.position.x - x_waypoints_list[0]) < 0.1 &&
                    abs(pos_drone.pose.position.y - y_waypoints_list[0]) < 0.1 &&
                    abs(pos_drone.pose.position.z - desire_height_)      < 0.1 )
                {
                    detected_flystate = LANDING;
                }
               
                pose.pose.position.x = x_waypoints_list[0];
                pose.pose.position.y = y_waypoints_list[0];
                pose.pose.position.z = desire_height_;
                local_pos_pub.publish(pose);
            }
                break;
            case LANDING:
            {
                // ROS_INFO("landing");
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("return -> AUTO.LAND enabled");
                }
            }
                break;
            
            default:
                ROS_ERROR("err!");
                break;
            }

            rate_detected.sleep();

            continue;   // 如果检测到aruco，下面的代码都不执行

        }
        
        // 正常巡航，若没有检测到，则在最后一个航点降落
        if (abs(pos_drone.pose.position.x - x_waypoints_list[count_points]) < 0.1 &&
            abs(pos_drone.pose.position.y - y_waypoints_list[count_points]) < 0.1 &&
            abs(pos_drone.pose.position.z - desire_height_)                 < 0.1 &&
            ros::Time::now() - last_request > ros::Duration(3.0) &&
            count_points < num_waypoints)
        {
            count_points += 1; // 发布下一个航点

            pose.pose.position.x = x_waypoints_list[count_points];
            pose.pose.position.y = y_waypoints_list[count_points];
            pose.pose.position.z = desire_height_;
            last_request = ros::Time::now();

            ROS_INFO("arriving at %dth waypoint, the next waypoint: x = %f, y = %f", count_points, x_waypoints_list[count_points], y_waypoints_list[count_points]);
        }

        if (count_points == num_waypoints)
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("the last waypoint AUTO.LAND enabled");
            }

        }
        else
        {
            local_pos_pub.publish(pose);
        }

        rate.sleep();
    }

    return 0;
}

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

geometry_msgs::PoseStamped aruco_pose;
void ArucoPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    aruco_pose = *msg;
    ROS_INFO("received pose: x = %f y = %f", aruco_pose.pose.position.x, aruco_pose.pose.position.y);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "aruco_pose_sub");
    ros::NodeHandle nh("~");

    ros::Subscriber aruco_pose_sub = nh.subscribe("/aruco_pose", 10, ArucoPoseCB);

    ros::spin();
    
    return 0;
}

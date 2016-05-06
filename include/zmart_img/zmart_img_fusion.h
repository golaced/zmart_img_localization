#ifndef __ZMART_IMG_FUSION_H__
#define __ZMART_IMG_FUSION_H__

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>

#include <iostream>
#include <cmath>
#include <std_msgs/Bool.h>

#include <opencv2/opencv.hpp>

#include <vo_flow/OpticalFlow.h>

using namespace std;
using namespace cv;

class ZmartImgFusion
{
public:
    ros::NodeHandle nh;
    ros::Subscriber flow_sub,imu_sub,lsd_sub,path_sub;
    ros::Publisher fusion_path_pub,fusion_velocity_pub;
    ros::Time t_last, t_now;
    geometry_msgs::PointStamped velocity_msg;

    nav_msgs::Path path_msg;
    geometry_msgs::PoseStamped pose_msgs;

    Mat A,B,C,Q,R,P,U,X,X_priori, Y,Rotation_b2w_a,Rotation_b2w_v;

    bool flow_ready;
    bool first_flag;
    int first_flag_counter;
    double flow_vx,flow_vy,lsd_x,lsd_y,dt,height_last,height,q_vicon[4],q[4],p[3],a[3],yaw,roll,pitch,bias_a[3];

public:
    ZmartImgFusion();
    ~ZmartImgFusion();
    void flowCallback(const vo_flow::OpticalFlow::ConstPtr msg);
    void q2rotation(double q[]);
    void imuCallback(const sensor_msgs::Imu::ConstPtr msg);
    void lsdlineCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
    void pathCallback(const geometry_msgs::PoseStamped::ConstPtr msg);
};

#endif // __ZMART_IMG_FUSION_H__


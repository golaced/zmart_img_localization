#ifndef __ZMART_IMG_LOCALIZATION_H__
#define __ZMART_IMG_LOCALIZATION_H__

#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <zmart_img/points.h>

#include <opencv2/opencv.hpp>

#include <vo_flow/OpticalFlow.h>
#include <math.h>

using namespace std;
using namespace cv;

class ZmartImgLocalization
{
public:
    ros::NodeHandle nh;

    ros::Subscriber imu_sub, point_sub, true_pose_sub, opticalflow_sub,vicon_pose_sub;

    ros::Publisher vicon_pose_path_pub,true_pose_path_pub,flow_pose_path_pub, imu_euler_pub, truth_euler_pub, line_pose_path_pub;

    geometry_msgs::Point temp3,temp4;
    nav_msgs::Path true_pose_path;
    nav_msgs::Path vicon_pose_path;
    nav_msgs::Path flow_pose_path;
    nav_msgs::Path line_pose_path;

    int f1,f2;
    double quality, dt;
    ros::Time flow_time_now,flow_time_last;
    double x_quater, y_quater, z_quater, w_quater;
    double imu_yaw, imu_roll, imu_pitch;
    double truth_yaw, truth_roll, truth_pitch;
    double fx,fy,cx,cy;
    double q[4],yaw;

    string ImuTopic, OpticalFlowTopic, TruePoseTopic,PointTopic,ViconPoseTopic;

    vector<Vec3d> flow_temp_;
    Vec3d flow_temp;

    Point2f ImgPoints;
    vector<Point2f> ImgPoints_;

    Point3d flow_path_temp;
    vector<Point3d> flow_path_temp_;

    Point3d flow_path,truth_path;

    vector<Point3f> objectPoints_;
    Point3f objectPoints;

    Mat xwyw,distortion_mat,intrinsic_mat,external_mat,camera_mat;
    Mat rotation_mat, rotation_vector,translation_mat;
    Mat Rotation_b2w_a,Rotation_b2w_v;
    vector<Point3d> flow_path_;

    bool flag;

    ZmartImgLocalization();
    ~ZmartImgLocalization();
    void q2rotation(double q[]);

    void viconpathCallback(const geometry_msgs::TransformStampedConstPtr& msg);
    void pathCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void opticalflowCallback(const vo_flow::OpticalFlowConstPtr& flow);
    void imuCallback(const sensor_msgs::ImuConstPtr& quater);
    void pointCallback(const zmart_img::pointsConstPtr& msg);
    void StateEstimation(double p, Point3d q);


};
#endif /* __ZMART_IMG_LOCALIZATION__ */

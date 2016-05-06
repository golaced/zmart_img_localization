#ifndef __BASE_FLOW_H__
#define __BASE_FLOW_H__

#include <iostream>
#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <vo_flow/OpticalFlow.h>

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class BaseFlow
{
public:
    ros::NodeHandle nh;
    ros::Subscriber image_sub,height_sub,imu_sub; //image,height,imu
    ros::Publisher flow_pub;//publish optical_flow

    Mat previous_image, current_image, display_image;

    vo_flow::OpticalFlow flow_msg;//user-define flow message

    int focus, filter_method, save_img;//filter_method=0,use weng_methnod, else use like_ransac

    ros::Time t_now, t_last;
    double dt, height;
    int height_counter;//?

    double q[4], a[3];//?

    string image_topic;
    bool have_pixhawk;

    Point3i weng_method(vector<Point3i> vec);
    Point3i like_ransac(vector<Point3i> vec);//?

    BaseFlow();
    ~BaseFlow();
    bool comp(const Point3i &a, const Point3i &b);

    virtual void imageCallback(const sensor_msgs::ImageConstPtr& msg) = 0;
    virtual void heightCallback(const sensor_msgs::RangeConstPtr& msg) =0;
    virtual void imuCallback(const sensor_msgs::ImuConstPtr& msg) = 0;//virtual function

};
#endif /*__BASE_FLOW_H__*/

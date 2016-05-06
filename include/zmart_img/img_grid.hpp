#ifndef  __IMG_GRID_HPP__
#define  __IMG_GRID_HPP__

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/opencv.hpp>
#include <zmart_img/points.h>

#include <iostream>
#include <sstream>
using namespace cv;
using namespace std;

class ImgGrid{

public:
    int cannyLowThreshold;
    int houghThreshold;
    int houghMinLineLength;
    int houghMaxLineGap;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    std::string ImgTopic;
    std::string calibFileName;
    cv::Mat dst, edge, gray;
    bool showImg;
    cv::vector<cv::Vec4i> lines;
    ros::Publisher intersection_pub;

public:
    ImgGrid(void);
    ~ImgGrid(void);

public:
    void imageCallback();

};





#endif // __IMG_GRID_HPP__






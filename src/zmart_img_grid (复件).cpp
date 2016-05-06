//#include <zmart_img/img_grid.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <opencv2/opencv.hpp>

#include <zmart_img/points.h>
#include <vo_flow/OpticalFlow.h>

#include <iostream>
#include <sstream>


#define HOUGH_RHO 1
#define HOUGH_THETA CV_PI/180
#define PI 3.1415926

using namespace std;
using namespace cv;

int cannyLowThreshold = 50;
int houghThreshold = 170;
int houghMinLineLength = 75.0;
int houghMaxLineGap = 35.0;

cv::Mat cameraMatrix;
cv::Mat distCoeffs;
std::string ImgTopic;
std::string calibFileName;
cv::Mat dst, edge, gray;
Mat display_image;
bool showImg;
cv::vector<cv::Vec4i> lines;

geometry_msgs::Point temp, temp_;
geometry_msgs::Point temp1;
geometry_msgs::Point temp2;
geometry_msgs::Point temp3;

zmart_img::points endpoint;
zmart_img::points endpoint1;
zmart_img::points endpoint2;
zmart_img::points endpoint3;
zmart_img::points endpoint4;

Mat xy,A,b;
double rad;
ros::Publisher endpoint_pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        dst.create(cv_ptr->image.size(),cv_ptr->image.type());
        /***** Edge Detection *****/
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        cv::namedWindow("image");
        display_image = cv_ptr->image.clone();
        cv::medianBlur(gray,edge,5);
        cv::Canny(edge,edge,cannyLowThreshold*2.5, cannyLowThreshold,3);

        /***** Lines Detection *****/
        cv::HoughLinesP(edge, lines, HOUGH_RHO, HOUGH_THETA, houghThreshold, houghMinLineLength, houghMaxLineGap);

/*****
        /***** Draw Results *****
        if (showImg)
        {
            dst = cv::Scalar::all(0);//initialization
            cv_ptr->image.copyTo(dst,edge);
            int line1 = 0;
            int line2 = 0;
            int line3 = 0;
            int line4 = 0;
            for (size_t i =0; i< lines.size(); i++)
            {
                cv::Vec4d line = lines[i];
                cv::line(dst,cv::Point(line[0],line[1]),cv::Point(line[2],line[3]),cv::Scalar(186,88,255),1,CV_AA);

            /***** Lines Filters *****
                if(abs(line[2]-line[0]) == 0 ||(abs(line[2]-line[0]) <= 10))
                {

                    line1++;
                    temp1.x = 1.0;
                    temp1.y = 0.0;
                    temp1.z = -line[0];
                    cv::line(display_image,cv::Point(temp1.z,0),cv::Point(temp1.z,480),cv::Scalar(100),2,CV_AA);
                    endpoint1.point.push_back(temp1);

                }
                else if (abs(line[3]-line[1])==0 || (abs(line[3]-line[1])<=10 ))
                {

                    line2++;
                    temp2.x = 0.0;
                    temp2.y = 1.0;
                    temp2.z = -line[1];
                    cv::line(display_image,cv::Point(0,temp2.z),cv::Point(640,temp2.z),cv::Scalar(100),2,CV_AA);
                    endpoint2.point.push_back(temp2);
                }
                else
                {
                    temp3.x = double ((line[3]-line[1])/(line[2]-line[0]));
                    temp3.z = double ((line[3]*line[0]-line[1]*line[2])/(line[0]-line[2]));
                    temp3.y = -1.0;
                    if (temp3.x > 0)
                    {
                        line3++;
                        cv::line(display_image,cv::Point(line[0],line[1]),cv::Point(line[2],line[3]),cv::Scalar(186,88,255),1,CV_AA);
                        string word1 = "k>0,start";
                        string word2 = "k>0,end";
                        putText(display_image, word1,cv::Point(line[0],line[1]),CV_FONT_BLACK, 0.3,cv::Scalar(0,255,0),2);
                        ////putText(display_image, word2, cv::Point(line[2],line[3]),CV_FONT_BLACK,0.3,cv::Scalar(0,255,0));
                        endpoint3.point.push_back(temp3);
                    }
                    else
                    {
                        line4++;
                        cv::line(display_image,cv::Point(line[0],line[1]),cv::Point(line[2],line[3]),cv::Scalar(0,0,255),1,CV_AA);
                        string word3 = "k<0,start";
                        string word4 = "k<0,end";
                        putText(display_image, word3, cv::Point(line[0],line[1]),CV_FONT_BLACK,0.3,cv::Scalar(255,0,0),2);
                        //putText(display_image, word4,cv::Point(line[2],line[3]),CV_FONT_BLACK,0.3, cv::Scalar(255,0,0));
                        endpoint4.point.push_back(temp3);
                    }

                    //cv::line(display_image,cv::Point(line[0],line[1]),cv::Point(line[2],line[3]),cv::Scalar(186,88,255),1,CV_AA);
                }
                //cout << "endpoint3: " << endpoint3 << endl;
                //cout << "endpoint4: " << endpoint4 << endl;
                //cout << "line3's quantity: " << line3 << endl;
                //cout << "line4's quantity: " << line4 << endl;
            }
            /**** caculate point ***
            for (int i = 0; i < line1; i++)
            {
                for (int j = 0; j< line2; j++)
                {
                    if (-endpoint1.point[i].z > 0 && -endpoint1.point[i].z < 640)
                        if(-endpoint2.point[j].z > 0 && -endpoint2.point[j].z < 480)
                        {
                            temp_.x = -endpoint1.point[i].z;
							temp_.y = -endpoint2.point[j].z;
							temp_.z = 1.0;
							//cout << "case1: x=a, y=b" << temp << endl;
							circle(display_image, Point(temp_.x,temp_.y),3,CV_RGB(0,0,0),5,8,0);
							endpoint.point.push_back(temp_);
                        }
                }
            }

            for (int i =0; i < line3; i++)
            {
                for (int j = 0; j < line4; j++)
                {
                    //temp.x = (endpoint3.point[i].y*endpoint4.point[j].z-endpoint4.point[j].y*endpoint3.point[i].z)/(endpoint3.point[i].x*endpoint4.point[j].y-endpoint4.point[j].x*endpoint3.point[i].y);//x=(b1*c2-b2*c1)/(a1*b2-a2*b1)
                    //temp.y = (endpoint4.point[j].x*endpoint3.point[i].z-endpoint3.point[i].x*endpoint4.point[j].z)/(endpoint3.point[i].x*endpoint4.point[j].y-endpoint4.point[j].x*endpoint3.point[i].y);//y=(a2*c1-a1*c2)/(a1*b2-a2*b1)
                    rad = abs(endpoint3.point[i].x-endpoint4.point[j].x)/abs(1+endpoint4.point[i].x*endpoint3.point[j].x);
                    //cout << "rad: " << rad << endl;
                    if ( rad > 5.67128)
                    {
                        A = (Mat_<double>(2,2) << endpoint3.point[i].x,endpoint3.point[i].y,endpoint4.point[j].x,endpoint3.point[j].y);
                        b = (Mat_<double>(2,1) << -1.0*endpoint3.point[i].z,-1.0*endpoint4.point[j].z);
                        //xy = Mat_<double>(2,1);
                        solve(A,b,xy,CV_SVD);
                        temp.x = xy.at<double>(0,0);
                        temp.y = xy.at<double>(1,0);
                        temp.z = 1.0;
                        if (temp.x > 0.0 && temp.x < 640.0)
                            if (temp.y >0.0 && temp.y < 480.0)
                            {
                                //cout << "case4: k1x-y+b1=0, k2x-y+b2=0 " << temp << endl;
                                circle(display_image, Point(temp.x,temp.y),3,CV_RGB(255,0,0),5,8,0);
                                endpoint.point.push_back(temp);
                            }
                    }

                }
            }

            endpoint1.point.clear();
			endpoint2.point.clear();
			endpoint3.point.clear();
			endpoint4.point.clear();
			endpoint.header.frame_id = "map";
			endpoint.header.stamp = msg -> header.stamp;
			//cout << "endpoint : " << endpoint << endl;
			endpoint_pub.publish(endpoint);
			endpoint.point.clear();
			//cout << "Number of Lines: " << lines.size() << endl;
            imshow("view",dst);
            imshow("image",display_image);
            waitKey(30);

        }
********/
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"zmart_img_grid");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    endpoint_pub = nh.advertise<zmart_img::points>("endpoint",10);
    nh_param.param("showImg",showImg,false);
    nh_param.param("ImgTopic",ImgTopic,string("camera/image_color"));
    if (showImg)
    {
        namedWindow("view");
        startWindowThread();

        cv::createTrackbar("CannyLT:", "view", &cannyLowThreshold, 120);
		cv::createTrackbar("HoughT:", "view", &houghThreshold, 400);
		cv::createTrackbar("HoughMLL:", "view", &houghMinLineLength, 150);
		cv::createTrackbar("HoughMLG:", "view", &houghMaxLineGap, 150);
    }

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe(ImgTopic,1,imageCallback);
    ros::spin();
    if(showImg)
        cv::destroyWindow("view");
    return 0;

}

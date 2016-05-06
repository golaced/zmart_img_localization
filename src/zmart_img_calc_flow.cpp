#include <iostream>
#include <std_msgs/Float32.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

#include <vo_flow/OpticalFlow.h>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

//#define Random(x) (rand() % x -1)

using namespace std;
using namespace Eigen;
using namespace cv;


int PIX_FOCUS = 240;
int IMAGE_COLS = 0;
int IMAGE_ROWS = 0;
int CORNER_THRE = 30;
int SAD_THRE = 30000;
int SEARCH_SIZE = 8;        // 8 maximum offset to search: 4 + 1/2 pixels  (better do not bigger than 8)
int NUM_BLOCKS = 5;         // 5 do not bigger than 8 ,do not less than 4

uint8_t qual = 0;
double dt = 0 , dt_optitrack = 0;
float pixel_flow_x = 0.0f, pixel_flow_y = 0.0f;
float meter_flow_x = 0.0f, meter_flow_y = 0.0f;
float pixel_flow_x_cor = 0.0f, pixel_flow_y_cor = 0.0f;
float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
float gyro_last_x = 0.0f, gyro_last_y = 0.0f, gyro_last_z = 0.0f;
float sum_x = 0, sum_y = 0;
float height = 0;
int path_clear_flag = 0 ;
int px_given = 0, py_given = 0;
int height_given = 60;
int yaw_given = 0;
int p = 15, d = 40;
int p_yaw = 10;
float error_yaw = 0;
float out_yaw = 0;
int pv = 35;

vector<Vec3d> flow_temp_;
int f1;
ros::Time flow_time_now,flow_time_last;
double dt2;
Point3d flow_path;
Point3d flow_path_temp;
/****************initialize********************/
Mat Q = (Mat_<double>(4,4) << 64,0,0,0,0,64,0,0,0,0,64,0,0,0,0,64);

Mat R = (Mat_<double>(2,2) << 36,0,0,36);

Mat P = (Mat_<double>(4,4) << 4,0,0,0,0,4,0,0,0,0,4,0,0,0,0,4);

Mat A = (Mat_<double>(4,4) << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1); //t

Mat B = (Mat_<double>(4,2) << 0,0,0,0,0,0,0,0); //t*t

Mat U = (Mat_<double>(2,1) << 0,0);

Mat C = (Mat_<double>(2,4) << 0,0,1,0,0,0,0,1);

Mat X = (Mat_<double>(4,1) << 0,0,0,0);

Mat X_priori = (Mat_<double>(4,1) << 0,0,0,0);

Mat Y = (Mat_<double>(2,1) << 0,0);

Mat II = Mat::eye(4, 4, CV_64FC1);

Mat acc_mat = (Mat_<float>(3,1) << 0,0,0);

float yaw_poweron,d_yaw;
ros::Time t_now , t_last , t_now_v , t_last_v;
Mat frame, previous_image, current_image;
uint8_t *image1, *image2;
Quaterniond q_imu;
Vector3d imu_euler;
Vector3d imu_ang;
Vector3d q_acc;
Vec3d flow_temp;
Mat imu_rotation;
geometry_msgs::PoseStamped optitrack_last_msg;
geometry_msgs::PointStamped optitrack_velocity_msg;
geometry_msgs::TwistStamped flow_gyro_msg;
geometry_msgs::TwistStamped flow_px4_msg;
geometry_msgs::PoseStamped pose_msg;
nav_msgs::Path flow_path_msg;
nav_msgs::Path optitrack_path_msg;
nav_msgs::Path vicon_path_msg;
nav_msgs::Path flow_pose_path;
ros::Publisher flow_gyro_pub , flow_path_pub,flow_pose_path_pub;
ros::Publisher optitrack_velocity_pub,  optitrack_path_pub, vicon_path_pub;
sensor_msgs::ImagePtr image_show_msgs;
image_transport::Publisher img_pub;

double quality;

bool first_opti = true;
bool first_flag_v = true;

uint8_t compute_flow(uint8_t *image1, uint8_t *image2, float *pixel_flow_x, float *pixel_flow_y);

Mat rotation_yaw(double yaw)
{
    Mat rotation = (Mat_<double>(3,3)<< cos(yaw),-sin(yaw),0,sin(yaw),cos(yaw),0,0,0,1);
    return rotation;
}

Vector3d quanternion2euler(Quaterniond q)
{
     Matrix3d m;
     double a = q.w(), b = q.x(), c = q.y(), d=q.z();
     m << a*a + b*b - c*c - d*d, 2*(b*c - a*d), 2*(b*d+a*c),
       2*(b*c+a*d), a*a - b*b + c*c - d*d, 2*(c*d - a*b),
       2*(b*d - a*c), 2*(c*d+a*b), a*a-b*b - c*c + d*d;
     double r = atan2(m(2,1), m(2,2));
     double p = asin(-m(2, 0));
     double y = atan2(m(1, 0), m(0, 0));
     Vector3d rpy(r, p, y);
     return rpy;
}

void flow_callback(const vo_flow::OpticalFlowConstPtr& msg)
{
    //cout << "opticalflow:: " << endl;
    flow_temp[0] = msg -> ground_distance + 0.3;
    flow_temp[2] = msg -> velocity_x;
    flow_temp[1] = msg -> velocity_y;
    quality = msg -> quality;

    //flow_px4_msg.header.stamp = ros::Time::now();
    flow_px4_msg.header.stamp = ros::Time::now();
    flow_px4_msg.twist.linear.x = flow_temp[2];
    flow_px4_msg.twist.linear.y = flow_temp[1];
    flow_gyro_pub.publish(flow_px4_msg);

    flow_temp_.push_back(flow_temp);
    //flow_time_temp_.push_back(flow_time);

    f1 = flow_temp_.size();
    //f2 = flow_time_temp_.size();

    //vector<Point3d>::iterator itr = flow_temp_.begin();
    if(f1 <= 1 || (flow_temp_[f1-1][1] == 0 && flow_temp_[f1-1][2] == 0))
    {
        //ROS_INFO("WARNING: NO flow_data!");
        flow_time_now = msg ->header.stamp;
        flow_time_last = flow_time_now;

    }
    else
    {
        flow_time_now = msg->header.stamp;
        dt2 = (double(flow_time_now.sec + 1e-9 * flow_time_now.nsec)) - (double(flow_time_last.sec + 1e-9 * flow_time_last.nsec));
        flow_path_temp.x = flow_temp_[f1-1][1] * dt;
        flow_path_temp.y = flow_temp_[f1-1][2] * dt;
        flow_path_temp.z = flow_temp_[f1-1][0];
        flow_time_last = flow_time_now;
        //cout << "flow_path_temp: " << flow_path_temp.x << " " <<flow_path_temp.y << endl;

        //flow_path_temp_.push_back(flow_path_temp);


        flow_path.x += flow_path_temp.x;
        flow_path.y += flow_path_temp.y;
        flow_path.z = flow_path_temp.z;

        //flow_path_.push_back(flow_path);
        cout << flow_path.x << " " << flow_path.y<<" " << flow_path.z << endl;
        geometry_msgs::PoseStamped temp2;
        temp2.pose.position.x = flow_path.x;
        temp2.pose.position.y = flow_path.y;
        temp2.pose.position.z = flow_path.z;

        //temp2.pose.orientation.x  = flow-> pose.orientation.x;
        //temp2.pose.orientation.y  = flow-> pose.orientation.y;
        //temp2.pose.orientation.z  = flow-> pose.orientation.z;
        //temp2.pose.orientation.w  = flow-> pose.orientation.w;
        temp2.header.stamp = msg-> header.stamp;
        flow_pose_path.poses.push_back(temp2);
        flow_pose_path.header.frame_id = "flow";
        flow_pose_path.header.stamp = msg-> header.stamp;
        flow_pose_path_pub.publish(flow_pose_path);
        //cout<< temp2.pose.position.x <<" " <<temp2.pose.position.y<<" "<< temp2.pose.position.z << endl;
    }
}

void imu_callback(const sensor_msgs::Imu::ConstPtr msg)
{
    imu_ang[0] = msg -> angular_velocity.x;
    imu_ang[1] = msg -> angular_velocity.y;
    imu_ang[2] = msg -> angular_velocity.z;
    q_imu.w() = msg -> orientation.w;
    q_imu.x() = msg -> orientation.x;
    q_imu.y() = msg -> orientation.y;
    q_imu.z() = msg -> orientation.z;
    q_acc[0] = msg->linear_acceleration.x;
    q_acc[1] = msg->linear_acceleration.y;
    q_acc[2] = msg->linear_acceleration.z;
    imu_euler = quanternion2euler(q_imu);


}
/***
void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr msg)
{
    geometry_msgs::PoseStamped temp;
    temp.pose.position.x = msg -> transform.translation.x;
    temp.pose.position.y = msg -> transform.translation.y;
    temp.pose.position.z = msg -> transform.translation.z;
    temp.header.stamp = msg->header.stamp;
    temp.header.frame_id = "flow";
    vicon_path_msg.poses.push_back(temp);
    vicon_path_msg.header.stamp = msg->header.stamp;
    vicon_path_msg.header.frame_id = "flow";
    vicon_path_pub.publish(vicon_path_msg);

    height = temp.pose.position.z;
}

****/

void optitrack_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
	optitrack_path_msg.header.stamp = msg->header.stamp;
	optitrack_path_msg.header.frame_id = "flow";
	optitrack_path_msg.poses.push_back(*msg);
	optitrack_path_pub.publish(optitrack_path_msg);

    height = msg -> pose.position.z;
	if(first_flag_v == true)
	{
		first_flag_v = false;
		t_now_v = msg->header.stamp;
		t_last_v = t_now_v;
		optitrack_last_msg = *msg;
	}
	else
	{
		t_now_v = msg->header.stamp;
		dt_optitrack = (double( t_now_v.sec + 1e-9 * t_now_v.nsec )) - (double( t_last_v.sec + 1e-9 * t_last_v.nsec ));
		t_last_v = t_now_v;

		optitrack_velocity_msg.header.stamp = msg->header.stamp;
		optitrack_velocity_msg.point.x = (msg->pose.position.x - optitrack_last_msg.pose.position.x)/dt_optitrack;
		optitrack_velocity_msg.point.y = (msg->pose.position.y - optitrack_last_msg.pose.position.y)/dt_optitrack;
		optitrack_velocity_pub.publish(optitrack_velocity_msg);
		optitrack_last_msg = *msg;
	}
}

void image_callback(const sensor_msgs::Image::ConstPtr msg)
{
    t_now = msg -> header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    frame = cv_ptr->image.clone();
    //cvPyrDown(const Arr*src, Arr*dst, int filter = CV_GAUSSIAN_5x5)
    pyrDown(frame, current_image, Size(frame.cols/2,frame.rows/2));
    if (!previous_image.data || !current_image.data)
	{
		printf("previous_image or current_image is empty !!!\n");
		previous_image = current_image.clone();
		t_last = t_now;
		IMAGE_COLS = current_image.cols;
		IMAGE_ROWS = current_image.rows;
		printf("current_image.cols:%d,current_image.rows:%d \n",IMAGE_COLS,IMAGE_ROWS);
        yaw_poweron = imu_euler[2];
		//gyro_last_x = rpy_[0];
		//gyro_last_y = rpy_[1];

	}
	else
	{
        image1 = &previous_image.at<uchar>(0,0);
        image2 = &current_image.at<uchar>(0,0);

        qual = compute_flow(image1, image2, &pixel_flow_x, &pixel_flow_y);

        d_yaw = imu_euler[2] - yaw_poweron;
        //imu_rotation = rotation_yaw(d_yaw);
        //acc_mat = (Mat_<float>(3,1) << q_acc[0], q_acc[1],q_acc[2]);
        //acc_mat = imu_rotation*acc_mat;
        dt = (double (t_now.sec + 1e-9 * t_now.nsec)) - (double (t_last.sec + 1e-9 * t_last.nsec));
        //gyro_x = rpy_[0];
        //gyro_y = rpy_[1];

        float pixel_flow_x0 = pixel_flow_x;
        float pixel_flow_y0 = pixel_flow_y;
        double gyro_flow_x = -1 * imu_ang[1]*dt*PIX_FOCUS;
        double gyro_flow_y = imu_ang[0]*dt*PIX_FOCUS;
        //cout << "gyro_flow_x" << gyro_x -gyro_last_x << endl;
        //cout << "gyro_flow_y" << gyro_y -gyro_last_y<< endl;
        // attitude compesation
        pixel_flow_x_cor = (pixel_flow_x0 - gyro_flow_x);
        pixel_flow_y_cor = (pixel_flow_y0 - gyro_flow_y);
        // if the yaw varies, must be corrected.

        float flow_x = cos(d_yaw)*pixel_flow_x_cor - sin(d_yaw)*pixel_flow_y_cor;
		float flow_y = sin(d_yaw)*pixel_flow_x_cor + cos(d_yaw)*pixel_flow_y_cor;

        //dimension reduction
        meter_flow_x = flow_x / PIX_FOCUS * height;
		meter_flow_y = flow_y / PIX_FOCUS * height;


        sum_x += 1 * meter_flow_y;
        sum_y += -1 * meter_flow_x;


/********************** EKF ***********************
		A.at<double>(0,2) = dt;
        A.at<double>(1,3) = dt;

        B.at<double>(0,0) = dt*dt*0.5;
        B.at<double>(1,1) = dt*dt*0.5;
        B.at<double>(2,0) = dt;
        B.at<double>(3,1) = dt;

		U.at<double>(0,0) = acc_mat.at<float>(0,0);
		U.at<double>(1,0) = acc_mat.at<float>(1,0);
		Y =(Mat_<double>(2,1) << meter_flow_x/dt, meter_flow_y/dt);

        X_priori = A*X + B*U;
        Mat P_priori = A*P*A.t() + Q;

		Mat K = P_priori*C.t()*((C*P_priori*C.t()+R).inv());  //DECOMP_LU   DECOMP_CHOLESKY
        X = (X_priori + K*(Y-C*X_priori));
        P = (II - K*C)*P_priori;
/*********************** EKF *************************


		if (path_clear_flag == 1)
        	{
			path_clear_flag = 0;
			sum_x = 0;
            sum_y = 0;
			X.at<double>(0,0) = sum_x;
			X.at<double>(1,0) = sum_y;
			X.at<double>(2,0) = 0;
			X.at<double>(3,0) = 0;
            flow_path_msg.poses.clear();
			optitrack_path_msg.poses.clear();
			yaw_poweron = yaw_poweron + d_yaw;
        	}

*************************************************/
        flow_gyro_msg.header.stamp = ros::Time::now();
        flow_gyro_msg.twist.linear.x = pixel_flow_x0;
        flow_gyro_msg.twist.linear.y = pixel_flow_y0;
        flow_gyro_msg.twist.linear.z = pixel_flow_x0 - gyro_flow_x;

        flow_gyro_msg.twist.angular.x = gyro_flow_x;
		flow_gyro_msg.twist.angular.y = gyro_flow_y;
		flow_gyro_msg.twist.angular.z = (pixel_flow_y0-gyro_flow_y);
		flow_gyro_pub.publish(flow_gyro_msg);

        flow_path_msg.header.stamp  = ros::Time::now();
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = sum_x;
        pose_msg.pose.position.y = sum_y;
        //pose_msg.pose.position.x = X.at<double>(0,0);
        //pose_msg.pose.position.y = X.at<double>(1,0);
        pose_msg.pose.position.z = height;
        flow_path_msg.poses.push_back(pose_msg);
        flow_path_pub.publish(flow_path_msg);

        image_show_msgs=cv_bridge::CvImage(std_msgs::Header(),"mono8",previous_image).toImageMsg();
        img_pub.publish(image_show_msgs);

		previous_image = current_image.clone();
		t_last = t_now;
	}

}

void* image_show_thread(void* dummy)
{
	printf("Entering image_show_thread ......");
	sleep(1);
	while(ros::ok())
	{
		namedWindow("param",0);
		createTrackbar( "corner_thre:", "param", &CORNER_THRE, 80, NULL );
		createTrackbar( "sad_thre:", "param", &SAD_THRE, 50000, NULL );
		createTrackbar( "search_size:", "param", &SEARCH_SIZE, 12, NULL );
		createTrackbar( "num_blocks:", "param", &NUM_BLOCKS, 8, NULL );

		createTrackbar( "path_clear_flag:", "param", &path_clear_flag, 1, NULL );
		createTrackbar( "px_given:", "param", &px_given, 2, NULL );
		createTrackbar( "py_given:", "param", &py_given, 2, NULL );
		createTrackbar( "h_given:", "param", &height_given, 100, NULL );
		createTrackbar( "yaw_given:", "param", &yaw_given, 360, NULL );
		createTrackbar( "    p:   ", "param", &p,        50, NULL );
		createTrackbar( "    d:   ", "param", &d,        50, NULL );
		createTrackbar( "    pv:  ", "param", &pv,       100, NULL );
		createTrackbar( "p_yaw:", "param", &p_yaw, 30, NULL );


		if(yaw_given == 360)
			yaw_given = 0;
		waitKey(30);  // imshow is too time-consuming
	}
}



int main(int argc, char *argv[])
{
	ros::init(argc,argv,"pixedflow_node");
	ros::NodeHandle nh("~");

    pthread_t image_show_thread_id; //线程句柄
    pthread_create(&image_show_thread_id,NULL,image_show_thread,NULL);

	image_transport::ImageTransport it(nh);
   	img_pub = it.advertise("image_show",1);
    flow_pose_path_pub = nh.advertise<nav_msgs::Path>("flow_px4_path",100);
	flow_gyro_pub = nh.advertise<geometry_msgs::TwistStamped>("flow_gyro_velocity",100);
	flow_path_pub = nh.advertise<nav_msgs::Path>("flow_path",100);
	optitrack_path_pub = nh.advertise<nav_msgs::Path>("/optitrack_path",10);
	//vicon_path_pub = nh.advertise<nav_msgs::Path>("/vicon_path",10);
	ros::Subscriber flow_sub = nh.subscribe("/serial_flow_msg",10,flow_callback);
	ros::Subscriber image_sub = nh.subscribe("/usb_cam/image_rect",1,image_callback);
	ros::Subscriber optitrack_sub = nh.subscribe("true_pose", 100 , optitrack_callback);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data",100,imu_callback);
    //ros::Subscriber vicon_sub = nh.subscribe("/vicon/firefly_sbx/firefly_sbx",100,vicon_callback);
	flow_gyro_msg.header.frame_id = "flow";
	flow_gyro_msg.header.stamp = ros::Time::now();
	flow_gyro_msg.twist.linear.x = 0;
	flow_gyro_msg.twist.linear.y = 0;
	flow_gyro_msg.twist.linear.z = 0;
	flow_gyro_msg.twist.angular.x = 0;
	flow_gyro_msg.twist.angular.y = 0;
	flow_gyro_msg.twist.angular.z = 0;

 	flow_path_msg.header.frame_id = "flow";
	flow_path_msg.header.stamp = ros::Time::now();

	ros::spin();
	return 0;
}


#include <zmart_img/zmart_img_localization.h>


using namespace std;
using namespace cv;

ZmartImgLocalization::ZmartImgLocalization():nh("~")
{
    ROS_INFO("Starting ZmartImgLocalization Node...");

    nh.param("ImuTopic",ImuTopic,string("mavros/imu/data"));
    nh.param("Opticalflow_Topic",OpticalFlowTopic,string("/serial_flow_msg"));
    nh.param("TruePose_Topic",TruePoseTopic,string("/true_pose"));
    nh.param("Point_Topic",PointTopic,string("/endpoint"));
    nh.param("ViconPose_Topic",ViconPoseTopic,string("/vicon/firefly_sbx/firefly_sbx"));
    opticalflow_sub = nh.subscribe(OpticalFlowTopic.c_str(),1,&ZmartImgLocalization::opticalflowCallback,this);
    imu_sub = nh.subscribe(ImuTopic.c_str(),1,&ZmartImgLocalization::imuCallback,this);
    true_pose_sub = nh.subscribe(TruePoseTopic.c_str(),1,&ZmartImgLocalization::pathCallback,this);
    vicon_pose_sub = nh.subscribe(ViconPoseTopic.c_str(),1,&ZmartImgLocalization::viconpathCallback,this);
    point_sub = nh.subscribe(PointTopic,1,&ZmartImgLocalization::pointCallback,this);

    vicon_pose_path_pub = nh.advertise<nav_msgs::Path>("vicon_pose_path",10);
    true_pose_path_pub = nh.advertise<nav_msgs::Path>("true_pose_path",10);
    flow_pose_path_pub = nh.advertise<nav_msgs::Path>("flow_pose_path",10);
    line_pose_path_pub = nh.advertise<nav_msgs::Path>("line_pose_path",10);

    imu_euler_pub = nh.advertise<geometry_msgs::Point>("imu_euler_rad",10);
    truth_euler_pub = nh.advertise<geometry_msgs::Point>("truth_euler_rad",10);

    flow_path = Point3d (0,0,0);
    intrinsic_mat = Mat::zeros(3,4,CV_64FC1);
    external_mat = Mat::zeros(4,4,CV_64FC1);

    rotation_mat = Mat::zeros(3,3,CV_64FC1);
    rotation_vector = Mat::zeros(3,1,CV_64FC1);
    translation_mat = Mat::zeros(3,1,CV_64FC1);
    //intrinsic_mat = (Mat_<double>(3,4)<<0,0,0,0,0,0,0,0,0,0,0,0);
    //external_mat = (Mat_<double>(4,4)<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);

    if(!nh.getParam("fx",fx))fx = 321.50692737558421;
	if(!nh.getParam("fy",fy))fy = 376.17915190396070;
	if(!nh.getParam("cx",cx))cx = 321.80781038990693;
	if(!nh.getParam("cy",cy))cy = 234.82795068297756;

    intrinsic_mat.at<double>(0,0) = fx;
    intrinsic_mat.at<double>(1,1) = fy;
    intrinsic_mat.at<double>(2,2) = 1.0;
    intrinsic_mat.at<double>(0,2) = cx;
    intrinsic_mat.at<double>(1,2) = cy;

    distortion_mat = Mat(5,1,CV_64FC1);
    distortion_mat.at<double>(0,0) = -2.9844534704723796e-01;
    distortion_mat.at<double>(1,0) = 7.0336622489752518e-02;
    distortion_mat.at<double>(2,0) = 0.0;
    distortion_mat.at<double>(3,0) = 0.0;
    distortion_mat.at<double>(4,0) = -6.1970798573996707e-03;


}

ZmartImgLocalization::~ZmartImgLocalization()
{
    ROS_INFO("Destroying ZmartImgLocalization...");
}

void ZmartImgLocalization::viconpathCallback(const geometry_msgs::TransformStampedConstPtr& msg)
{
	geometry_msgs::PoseStamped temp2;
	temp2.pose.position.x = msg-> transform.translation.x;
	temp2.pose.position.y = msg-> transform.translation.y;
	temp2.pose.position.z = msg-> transform.translation.z;
	truth_path.x = temp2.pose.position.x;
	truth_path.y = temp2.pose.position.y;
	truth_path.z = temp2.pose.position.z;
	temp2.pose.orientation.x  = msg-> transform.rotation.x;
	temp2.pose.orientation.y  = msg-> transform.rotation.y;
	temp2.pose.orientation.w  = msg-> transform.rotation.w;
	temp2.header.stamp = msg-> header.stamp;
	temp2.header.frame_id = "map";
	vicon_pose_path.poses.push_back(temp2);
	vicon_pose_path.header.frame_id = "map";
	vicon_pose_path.header.stamp = msg-> header.stamp;
	vicon_pose_path_pub.publish(vicon_pose_path);
}
void ZmartImgLocalization::pathCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    //ROS_INFO("INFO: path");
	geometry_msgs::PoseStamped temp1;
	temp1.pose.position.x = msg-> pose.position.x;
	temp1.pose.position.y = msg-> pose.position.y;
	temp1.pose.position.z = msg-> pose.position.z;
	truth_path.x = temp1.pose.position.x;
	truth_path.y = temp1.pose.position.y;
	truth_path.z = temp1.pose.position.z;
	temp1.pose.orientation.x  = msg-> pose.orientation.x;
	temp1.pose.orientation.y  = msg-> pose.orientation.y;
	temp1.pose.orientation.z  = msg-> pose.orientation.z;
	temp1.pose.orientation.w  = msg-> pose.orientation.w;
	temp1.header.stamp = msg-> header.stamp;
	temp1.header.frame_id = "map";
	true_pose_path.poses.push_back(temp1);
	true_pose_path.header.frame_id = "map";
	true_pose_path.header.stamp = msg-> header.stamp;
	true_pose_path_pub.publish(true_pose_path);
	//cout << temp1.pose.position.x <<" "<< temp1.pose.position.y <<" "<< temp1.pose.position.z << " "<< temp1.pose.orientation.x << " " << temp1.pose.orientation.y << " " << temp1.pose.orientation.z << " " << temp1.pose.orientation.w << endl;
    truth_roll = atan2(2*(temp1.pose.orientation.w*temp1.pose.orientation.x + temp1.pose.orientation.y*temp1.pose.orientation.z), 1.0-2*(temp1.pose.orientation.x*temp1.pose.orientation.x + temp1.pose.orientation.y*temp1.pose.orientation.y));
	truth_pitch = asin(2*(temp1.pose.orientation.w*temp1.pose.orientation.y - temp1.pose.orientation.z*temp1.pose.orientation.x));
	truth_yaw = atan2 (2*(temp1.pose.orientation.w*temp1.pose.orientation.z+temp1.pose.orientation.x*temp1.pose.orientation.y), 1.0-2*(temp1.pose.orientation.y*temp1.pose.orientation.y + temp1.pose.orientation.z*temp1.pose.orientation.z));
	temp4.x = truth_roll;
	temp4.y = truth_pitch;
	temp4.z = truth_yaw + 0.000498176;
	//cout << "truth_yaw: " << truth_yaw << endl;
	truth_euler_pub.publish(temp4);


}

void ZmartImgLocalization::opticalflowCallback(const vo_flow::OpticalFlowConstPtr& flow)
{
    //cout << "opticalflow:: " << endl;
    flow_temp[0] = flow -> ground_distance + 0.3;
    flow_temp[2] = flow -> velocity_x;
    flow_temp[1] = flow -> velocity_y;
    quality = flow -> quality;


    flow_temp_.push_back(flow_temp);
    //flow_time_temp_.push_back(flow_time);

    f1 = flow_temp_.size();
    //f2 = flow_time_temp_.size();

    //vector<Point3d>::iterator itr = flow_temp_.begin();
    if(f1 <= 1 || (flow_temp_[f1-1][1] == 0 && flow_temp_[f1-1][2] == 0))
    {
        //ROS_INFO("WARNING: NO flow_data!");
        flow_time_now = flow ->header.stamp;
        flow_time_last = flow_time_now;

    }
    else
    {
        flow_time_now = flow->header.stamp;
        dt = (double(flow_time_now.sec + 1e-9 * flow_time_now.nsec)) - (double(flow_time_last.sec + 1e-9 * flow_time_last.nsec));
        flow_path_temp.x = flow_temp_[f1-1][1] * dt;
        flow_path_temp.y = flow_temp_[f1-1][2] * dt;
        flow_path_temp.z = flow_temp_[f1-1][0];
        flow_time_last = flow_time_now;

        //flow_path_temp_.push_back(flow_path_temp);


        flow_path.x += flow_path_temp.x;
        flow_path.y += flow_path_temp.y;
        flow_path.z = flow_path_temp.z;

        //flow_path_.push_back(flow_path);
        //cout << "flow_path: " << flow_path << endl;
        geometry_msgs::PoseStamped temp2;
        temp2.pose.position.x = flow_path.x;
        temp2.pose.position.y = flow_path.y;
        temp2.pose.position.z = flow_path.z;

        //temp2.pose.orientation.x  = flow-> pose.orientation.x;
        //temp2.pose.orientation.y  = flow-> pose.orientation.y;
        //temp2.pose.orientation.z  = flow-> pose.orientation.z;
        //temp2.pose.orientation.w  = flow-> pose.orientation.w;
        temp2.header.stamp = flow-> header.stamp;
        flow_pose_path.poses.push_back(temp2);
        flow_pose_path.header.frame_id = "map";
        flow_pose_path.header.stamp = flow-> header.stamp;
        flow_pose_path_pub.publish(flow_pose_path);
        //cout<< temp2.pose.position.x <<" " <<temp2.pose.position.y<<" "<< temp2.pose.position.z << endl;
    }


}


void ZmartImgLocalization::imuCallback(const sensor_msgs::ImuConstPtr& quater)
{
    x_quater = quater -> orientation.x;
	y_quater = quater -> orientation.y;
	z_quater = quater -> orientation.z;
	w_quater = quater -> orientation.w;
	//cout << "w_quater: " << w_quater << endl;
	imu_roll = atan2(2*(w_quater*x_quater + y_quater*z_quater), 1.0-2*(x_quater*x_quater + y_quater*y_quater));
	imu_pitch = asin(2*(w_quater*y_quater - z_quater*x_quater));
	imu_yaw = atan2 (2*(w_quater*z_quater+x_quater*y_quater), 1.0-2*(y_quater*y_quater + z_quater*z_quater));
	//cout << imu_roll <<" " << imu_pitch << " " << imu_yaw << endl;
	temp3.x = imu_roll;
	temp3.y = imu_pitch;
	temp3.z = imu_yaw - 0.728966;
	imu_euler_pub.publish(temp3);


}


void ZmartImgLocalization::pointCallback(const zmart_img::pointsConstPtr& msg)
{
    //ROS_INFO("INFO: Point");
    geometry_msgs::PoseStamped temp4;
    geometry_msgs::PoseStamped temp5;

    StateEstimation(imu_yaw,truth_path);
    //cout << "external_mat " << external_mat << endl;

    camera_mat = intrinsic_mat * external_mat;

    //cout << "Camera_mat: " << camera_mat << endl;

    if (msg->point.size() <= 4)
    {
        //cout << "Warning: The size of point is zero ... " << endl;
        line_pose_path_pub.publish(flow_pose_path);
        //cout << "line_path: " << flow_path << endl;
    }
    else
    {
        for (int i = 0; i < msg->point.size(); i++)
        {
            ImgPoints.x = float(msg->point[i].x);
            ImgPoints.y = float(msg->point[i].y);

            ImgPoints_.push_back(ImgPoints);
            //cout << "Imgpoints: " << ImgPoints_ << endl;
            Mat solve_b = (Mat_<double>(2,1) << ImgPoints_[i].x - camera_mat.at<double>(0,3),ImgPoints_[i].y - camera_mat.at<double>(1,3));
            //cout << "b: " << solve_b  << endl;
            solve(camera_mat(Range(0,2),Range(0,2)),solve_b,xwyw,CV_SVD);
            //xwyw.at<double>(0,0) +=0.6;
            //xwyw.at<double>(1,0) -=0.75;
            //cout<< "xwyw: " << xwyw << endl;
            //solve(camera_mat(Range(0,2),Range(0,2)),solve_b,xwyw2,CV_LU);
            cout<< "xwyw: " << xwyw << endl;
            objectPoints.x = float(xwyw.at<double>(0,0));
            objectPoints.y = float(xwyw.at<double>(1,0));
            objectPoints.z = float(0.0);
            objectPoints_.push_back(objectPoints);

        }
        //Rodrigues(external_mat(Range(0,3),Range(0,3)),rotation_mat);
        //translation_mat = (Mat_<double>(3,0)<< flow_path.x, flow_path.y, flow_path.z);
        //cout << "imagepoints: " << ImgPoints_ << endl;
        //cout << "objectpoints: " << objectPoints_ << endl;
        //cout << "distortion_mat: " << distortion_mat << endl;
        solvePnP(objectPoints_,ImgPoints_,intrinsic_mat(Range(0,3),Range(0,3)),distortion_mat,rotation_vector, translation_mat,false);
        Rodrigues(rotation_vector,rotation_mat);
        translation_mat = rotation_mat.t()*translation_mat;
        cout << "translation_mat: " << translation_mat << endl;

        temp5.pose.position.x = translation_mat.at<double>(0,0);
        temp5.pose.position.y = translation_mat.at<double>(1,0);
        temp5.pose.position.z = translation_mat.at<double>(2,0);
        temp5.header.stamp = msg-> header.stamp;
        line_pose_path.poses.push_back(temp5);
        line_pose_path.header.frame_id = "map";
        line_pose_path.header.stamp = msg-> header.stamp;
        line_pose_path_pub.publish(line_pose_path);
        objectPoints_.clear();
        ImgPoints_.clear();
        //cout << "rotation_mat: " << rotation_mat << endl;
        //cout << "translation_mat: " << translation_mat << endl;
    }

}


void ZmartImgLocalization::q2rotation(double q[])
{
    double q00 = q[0]*q[0];   double q01 = q[0]*q[1];   double q02 = q[0]*q[2];    double q03 = q[0]*q[3];
                              double q11 = q[1]*q[1];   double q12 = q[1]*q[2];    double q13 = q[1]*q[3];
                                                        double q22 = q[2]*q[2];    double q23 = q[2]*q[3];
                                                                                   double q33 = q[3]*q[3];

    Rotation_b2w_a = (Mat_<double>(3,3) << q00+q11-q22-q33,    2*(q12-q03),    2*(q13+q02),
                                           2*(q12+q03),     q00-q11+q22-q33,   2*(q23-q01),
                                           2*(q13-q02),     2*(q23+q01),    q00-q11-q22+q33);

    yaw = atan( 2*(q03 + q12)/(1-2*(q33+q22)) ) - 0.728966;
    //std::cout << "yaw"<<yaw*180/3.14 << std::endl;

    Rotation_b2w_v = (Mat_<double>(2,2) << cos(yaw),-1*sin(yaw),sin(yaw),cos(yaw));
}


void ZmartImgLocalization::StateEstimation(double p, Point3d q)
{

    external_mat.at<double>(0,0) = cos(p);//yaw　denotes of rad
    external_mat.at<double>(0,1) = sin(p);
    external_mat.at<double>(1,0) = -1*sin(p);
    external_mat.at<double>(1,1) = cos(p);
    external_mat.at<double>(2,2) = 1.0;
    external_mat.at<double>(0,3) = cos(p)*q.x-sin(p)*q.y;
    external_mat.at<double>(1,3) = sin(p)*q.x+cos(p)*q.y;
    external_mat.at<double>(2,3) = q.z-0.3;
    external_mat.at<double>(3,3) = 1.0;

}






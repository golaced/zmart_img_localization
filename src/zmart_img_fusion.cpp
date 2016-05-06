#include <zmart_img/zmart_img_fusion.h>

ZmartImgFusion::ZmartImgFusion():nh("~")
{
    ROS_INFO("Starting ZmartImgFusin Node...");
    string flow_topic;
    nh.param("flow_topic",flow_topic,string("/path_pose"));
    flow_sub = nh.subscribe("/serial_flow_msg",1,&ZmartImgFusion::flowCallback, this);
    lsd_sub = nh.subscribe(flow_topic.c_str(),1,&ZmartImgFusion::lsdlineCallback, this);
    path_sub = nh.subscribe("/true_pose",1,&ZmartImgFusion::pathCallback,this);
    imu_sub = nh.subscribe("/mavros/imu/data",1,&ZmartImgFusion::imuCallback,this);

    fusion_path_pub = nh.advertise<nav_msgs::Path>("/fusion_path",10);
    fusion_velocity_pub = nh.advertise<geometry_msgs::PointStamped>("/fusion_velocity",10);

    flow_ready = false;
    first_flag = true;
    first_flag_counter = 0;
    flow_vx = 0;
    flow_vy = 0;

    Q = (Mat_<double>(4,4) << 64,0,0,0,0,64,0,0,0,0,64,0,0,0,0,64);

    R = (Mat_<double>(4,4) << 36000,0,0,0,0,36000,0,0,0,0,64,0,0,0,0,360);

    P = (Mat_<double>(4,4) << 4,0,0,0,0,4,0,0,0,0,4,0,0,0,0,4);

    A = (Mat_<double>(4,4) << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1); //t

    B = (Mat_<double>(4,2) << 0,0,0,0,0,0,0,0); //t*t

    U = (Mat_<double>(2,1) << 0,0);

    //C = (Mat_<double>(2,4) << 1,0,0,0,0,1,0,0);
    C = (Mat_<double>(4,4) << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);

    X = (Mat_<double>(4,1) << 0,0,0,0);

    X_priori = (Mat_<double>(4,1) << 0,0,0,0);

    Y = (Mat_<double>(4,1) << 0,0,0,0);

    Rotation_b2w_a = (Mat_<double>(3,3) << 1,0,0,0,1,0,0,0,1);
    Rotation_b2w_v = (Mat_<double>(2,2) << 1,0,0,1);
}

ZmartImgFusion::~ZmartImgFusion()
{
    ROS_INFO("Destroying ZmartImgFusion...");
}

void ZmartImgFusion::pathCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{

	//cout <<"coordinate_truepath:" << temp.pose.position << endl;
	p[0] = msg -> pose.position.x;
	p[1] = msg -> pose.position.y;
	p[2] = msg -> pose.position.z;
    q_vicon[1] = msg-> pose.orientation.x;
    q_vicon[2]= msg-> pose.orientation.y;
    q_vicon[3]= msg-> pose.orientation.z;
    q_vicon[0]= msg-> pose.orientation.w;
    //cout << q_vicon[1] << " " << q_vicon[2] << " " << q_vicon[3] << " " << q_vicon[0] << " ";
    q2rotation(q_vicon);
    //cout <<roll <<" " <<pitch << " "<<yaw << " ";

}
void ZmartImgFusion::imuCallback(const sensor_msgs::Imu::ConstPtr msg)
{
    if (first_flag == true)
    {
        t_last = msg->header.stamp;
        first_flag_counter ++;
        if(first_flag_counter > 10)
        {
            first_flag_counter = 0;
            first_flag = false;
            bias_a[0] = msg->linear_acceleration.x;
            bias_a[1] = msg->linear_acceleration.y;
            bias_a[2] = msg->linear_acceleration.z;
        }
        //cout << "/* message */" << first_flag_counter << endl;
    }

    else
    {
        t_now = msg->header.stamp;
        dt = (double( t_now.sec + 1e-9 * t_now.nsec )) - (double( t_last.sec + 1e-9 * t_last.nsec ));
        t_last = t_now;
        q[1] = msg->orientation.x + 0.005;
        q[2] = msg->orientation.y + 0.020;
        q[3] = msg->orientation.z + 0.356;
        q[0] = msg->orientation.w + 0.065;

        //cout << "q_imu:" << q[1] << " " << q[2] << " " << q[3] << " " << q[0] << endl;
        q2rotation(q);
        //cout << roll <<" " <<pitch << " "<<yaw << " ";

        a[0] = msg->linear_acceleration.x;
        a[1] = msg->linear_acceleration.y;
        a[2] = msg->linear_acceleration.z;

        Mat a_body = (Mat_<double>(3,1) << a[0],a[1],a[2]);
        //this means accleration is denoted in body frame
        Mat a_inertial = Rotation_b2w_a*a_body;
        //Rotation_b2w_a: rotation matrix
        //cout << "a_inertial: " << a_inertial << endl;
         U = (Mat_<double>(2,1) << a_inertial.at<double>(0,0),a_inertial.at<double>(1,0));
        // given the input a_intertial

        A.at<double>(0,2) = dt;
        A.at<double>(1,3) = dt;

        B.at<double>(0,0) = dt*dt*0.5;
        B.at<double>(1,1) = dt*dt*0.5;
        B.at<double>(2,0) = dt;
        B.at<double>(3,1) = dt;
        //xk = A xk-1 + B uk

        /***** prediction *****/
        X_priori = A*X + B*U;
        //state equation, update the system state x(k|k-1)
        Mat P_priori = A*P*A.t() + Q;
        //update the covariance relative to x(k|k-1)
        Mat II = Mat::eye(4, 4, CV_64FC1);

        //flow_ready = false

        if (flow_ready == true)
        {
            flow_ready = false;
            Mat v_body = (Mat_<double>(2,1) << flow_vx,flow_vy);
            //Mat v_inertial = Rotation_b2w_v*v_body;
            Mat d_inertial = (Mat_<double>(2,1) << lsd_x,lsd_y);
            //Y = (Mat_<double>(2,1) << v_inertial.at<double>(0,0),-1*v_inertial.at<double>(1,0));
            Y = (Mat_<double>(4,1) << lsd_x,lsd_y,flow_vx,flow_vy);
           // std::cout << v_body<<"Y:"<<Y << std::endl;

            //Y = v_body.clone();
            /***** measure values *****/
            Mat K = P_priori*C.t()*((C*P_priori*C.t()+R).inv());
            //cout << "K" << K << endl;
            //figure out the kalman gain
            //K(k)= P(k|k-1)*C^T/(C*P(k|k-1)*C^T+R)
            X = (X_priori + K*(Y-C*X_priori));
            //combine the predictive value and measurement value, figure out the optimial value x(k|k)
            P = (II - K*C)*P_priori;
            //cout << "P" << P << endl;
            //update the covariance relative to x(k|k)
            path_msg.header.stamp = msg->header.stamp;
            path_msg.header.frame_id = "map";
            pose_msgs.pose.position.x = X.at<double>(0,0);
            pose_msgs.pose.position.y = X.at<double>(1,0);
            //px,py
            pose_msgs.pose.position.z = height;
            //cout << pose_msgs.pose.position.x << " " << pose_msgs.pose.position.y << " " << pose_msgs.pose.position.z << " ";
            path_msg.poses.push_back(pose_msgs);

            fusion_path_pub.publish(path_msg);

            velocity_msg.point.x =  X.at<double>(2,0);
            velocity_msg.point.y =  X.at<double>(3,0);
            //vx,vy
            velocity_msg.header.stamp = msg->header.stamp;
            velocity_msg.header.frame_id = "map";
            fusion_velocity_pub.publish(velocity_msg);
        }
        else
        {
            X = X_priori.clone();
            P = P_priori.clone();

            path_msg.header.stamp = msg->header.stamp;
            path_msg.header.frame_id = "map";
            pose_msgs.pose.position.x = X.at<double>(0,0);
            pose_msgs.pose.position.y = X.at<double>(1,0);
            pose_msgs.pose.position.z = height;
            path_msg.poses.push_back(pose_msgs);
            fusion_path_pub.publish(path_msg);


            velocity_msg.point.x =  X.at<double>(2,0);
            velocity_msg.point.y =  X.at<double>(3,0);
            velocity_msg.header.stamp = msg->header.stamp;
            velocity_msg.header.frame_id = "map";
            fusion_velocity_pub.publish(velocity_msg);

        }
         cout << pose_msgs.pose.position.x - p[0] << " " << pose_msgs.pose.position.y - p[1]<< " " << pose_msgs.pose.position.z -p[3]<< endl;
    }
}

//solve the quantion to euler
void ZmartImgFusion::q2rotation(double q[])
{
    double q00 = q[0]*q[0];   double q01 = q[0]*q[1];   double q02 = q[0]*q[2];    double q03 = q[0]*q[3];
                              double q11 = q[1]*q[1];   double q12 = q[1]*q[2];    double q13 = q[1]*q[3];
                                                        double q22 = q[2]*q[2];    double q23 = q[2]*q[3];
                                                                                   double q33 = q[3]*q[3];

    Rotation_b2w_a = (Mat_<double>(3,3) << q00+q11-q22-q33,    2*(q12-q03),    2*(q13+q02),
                                           2*(q12+q03),     q00-q11+q22-q33,   2*(q23-q01),
                                           2*(q13-q02),     2*(q23+q01),    q00-q11-q22+q33);

    roll= atan2(2*(q01+q23),(1-2*(q11+q22)));
    pitch=asin(2*(q02-q13)) ;
    yaw = atan2( 2*(q03 + q12),(1-2*(q33+q22)) );
    //std::cout << "yaw"<<yaw*180/3.14 << std::endl;

    Rotation_b2w_v = (Mat_<double>(2,2) << cos(yaw),-1*sin(yaw),sin(yaw),cos(yaw));
}


void ZmartImgFusion::flowCallback(const vo_flow::OpticalFlow::ConstPtr msg)
{
    flow_ready = true;
    height = msg->ground_distance;
    flow_vx = msg->velocity_x;
    flow_vy = msg->velocity_y;
}

void ZmartImgFusion::lsdlineCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    //flow_ready = true;
    //height = msg -> pose.position.z;
    lsd_x = msg -> pose.position.x;
    lsd_y = msg -> pose.position.y;

}

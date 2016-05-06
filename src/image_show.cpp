#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <math.h>

using namespace std;
using namespace cv;


int image_saving = 0;

Mat previous_image, frame_temp;

void image_callback(const sensor_msgs::Image::ConstPtr msg)
{
	cv_bridge::CvImagePtr cv_ptr;
    	cv_ptr = cv_bridge::toCvCopy(msg , sensor_msgs::image_encodings::MONO8);
	previous_image = cv_ptr -> image.clone();

	//resize(previous_image, frame_temp, Size(), 3, 3, CV_INTER_AREA);
	pyrUp(previous_image, frame_temp, Size(previous_image.cols*2,previous_image.rows*2));
	pyrUp(frame_temp, previous_image, Size(frame_temp.cols*2,frame_temp.rows*2));

	imshow("compute_flow_image", previous_image);

	createTrackbar( "image_saving:", "compute_flow_image", &image_saving, 1, NULL );

	if(image_saving == 1)
	{
		image_saving = 0;
		printf("saving image ......\n");
		int rand_id = (int)(1000.0*rand()/RAND_MAX+1.0);
		char filename[100];
		sprintf(filename, "/home/exbot/image/%d.jpg", rand_id);
		printf("image_url: %s\n",filename);
		imwrite(filename,previous_image);
	}

	waitKey(1); //  tuxinghua time-consuming
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"image_show_node");
	ros::NodeHandle nh("~");
	ros::Subscriber image_sub = nh.subscribe("/fixedflow_node/image_show",10,image_callback);
	namedWindow("compute_flow_image",WINDOW_NORMAL);

	ros::spin();
	return 0;
}


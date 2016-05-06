#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <sstream>
#include <fstream>//fstream,对打开的文件进行读写操作
#include <opencv2/opencv.hpp>
#include <vector>
#include <cxcore.h>//该库包括opencv最基本的数据结构，包括矩阵，数组的基本运算

using namespace std;

int cannyLowThreshold = 67;
int HoughPThreshold=175;
int minLineLength=65;
int maxLineGap=65;
int HoughThreshold=206;
int stn=0;
int srn=0;
fstream dataFile, dataFile2;
int frame=0;
cv::vector<cv::Vec4i> lines;
cv::vector<cv::Vec2i> insec;

void lineineract(cv::vector<cv::Vec4i> lines)
{              
	insec.clear();
	for( size_t j=0;  j<lines.size()-1; j++)
	{            
		int x1, x2, x3, x4, y1, y2, y3, y4;
		int x0, y0;
		cv::Point pt1, pt2;
		cv::Vec4i line1 = lines[j];
		
		x1 = cvRound(line1[0]);
		y1 = cvRound(line1[1]);
		x2 = cvRound(line1[2]);
		y2 = cvRound(line1[3]);


	              for( size_t k=j+1;  k<lines.size(); k++)
		{
			cv::Vec4i line2 = lines[k];
			cv::Vec2i pointemp;
			cv::Point pt3, pt4;
		             x3 = cvRound(line2[0]);
		             y3 = cvRound(line2[1]);
		             x4 = cvRound(line2[2]);
		             y4 = cvRound(line2[3]);		
		            
		            if((((y1-y2)*(x3-x4)-(y3-y4)*(x1-x2)) != 0) && ((x1-x2) != 0) )
		                {
		                    x0=((y4-y2)*(x1-x2)*(x3-x4)+(y1-y2)*(x3-x4)*x2-(y3-y4)*(x1-x2)*x4)/((y1-y2)*(x3-x4)-(y3-y4)*(x1-x2));
		                    y0=(y1-y2)*(x0-x2)/(x1-x2)+y2;
		                }
		            else
		            {
		            	cout << "Error" << endl;
		            	continue;
		            }

		            pointemp[0] = cvRound(x0);
		            pointemp[1] = cvRound(y0);
		            insec.push_back(pointemp);
		}
	}
}

/*cv::Mat*/ void drawinsec(cv::vector<cv::Vec2i> points, cv::Mat image)
{            
	//cv::Mat paper;
	cv::Point pt;
	//paper.create(image.size(), image.type());
	//paper = cv::Scalar::all(0);
	//paper = image;  
	for(size_t i=0;  i<points.size(); i++)
	{
		pt=points[i];
		cv::circle( image, pt, 8, CV_RGB(255,0,0), 1,CV_AA, 0);
		//cv::cvtColor(paper1, paper2, cv::COLOR_BGR2GRAY); 
	}
            // return paper;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{            
	
	double k, k1, k2;
	double c, c1, c2;
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

	cv::Mat dst, edge, gray, imageROI, paper, show;
	//cv::Mat dst2;
	cv::Rect rect;

	rect.x=120;
	rect.y=90;
	rect.width =400;
	rect.height=300;

	//cut.create(200, 200, cv_ptr->image.type());
              //cv::cvSetImageROI(cv_ptr->image,rect);
              //cv::cvCopy(cv_ptr->image,cut);
              //cv::cvResetImage(cv_ptr->image);

              imageROI=cv_ptr->image(rect);         
              cv::rectangle( cv_ptr->image, rect, cv::Scalar(0, 0, 255), 1, CV_AA ); 

              //cout << cv_ptr->image.size() << cv_ptr->image.type() << imageROI.size() << endl;

              dst.create(imageROI.size(), imageROI.type());
	cv::cvtColor(imageROI, gray, cv::COLOR_BGR2GRAY);   // now use the central part of the image. 
	cv::medianBlur(gray, edge, 5);
	cv::Canny(edge, edge, cannyLowThreshold*2.5, cannyLowThreshold, 3);

	dst = cv::Scalar::all(0);  
	imageROI.copyTo(dst, edge);       // dst is the target image(after canny)

              //dst2=edge;
 	
	//cv::vector<cv::Vec2f> lines;
	//HoughLines(edge,  lines, 1, CV_PI/180, HoughThreshold, srn/100, stn/100);

             HoughLinesP(edge,  lines, 1, CV_PI/180, HoughPThreshold, minLineLength, maxLineGap);
             
             dataFile2 << "frame:" << frame++ << endl;
	for( size_t i=0;  i<lines.size(); i++)
	{
		/*float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0=a*rho, y0=b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));
		line(  cv_ptr->image, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA );
		dataFile<< "(" << pt1.x << "," << pt1.y << ") ; " << "(" << pt2.x << "," << pt2.y << ")" << endl;*/
		
		cv::Point pt1, pt2;
		cv::Vec4i l = lines[i];
		pt1.x = cvRound(l[0]);
		pt1.y = cvRound(l[1]);
		pt2.x = cvRound(l[2]);
		pt2.y = cvRound(l[3]);
		line(  dst, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA );
		line(  imageROI, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA );
		//line(  cv_ptr->image, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA );
                            k1= -(pt1.x-pt2.x);
                            k2= (pt1.y-pt2.y);
                            k = k1/k2;
                            c1 = pt1.x*pt2.y-pt2.x*pt1.y;
                            c2 = pt1.y-pt2.y;
                            c = c1/c2;
		dataFile << "(" << pt1.x << "," << pt1.y << ") ; " << "(" << pt2.x << "," << pt2.y << ")" << endl;
		dataFile2 << k << " , " << c << endl;
		
		/*line(  dst2, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(186,88,255), 1, CV_AA );
		dataFile<< "(" << cv::Point(l[0], l[1]) << "," << cv::Point(l[2], l[3])  << endl;*/
	}
              show=imageROI; 
              lineineract(lines);
              /*paper =*/drawinsec(insec, imageROI);
              //imageROI.copyTo(show);
              //imageROI = cv::Scalar::all(0);  
              //.copyTo(imageROI, paper);

              dataFile<< "end of this frame" << endl;
	cv::imshow("Line abstract", dst);
	cv::imshow("Original image", cv_ptr->image);
	cv::imshow("Region of Interest", imageROI);
	//cv::imshow("Draw Point", paper);

	cv::waitKey(30);
}

int main(int argc, char* argv[])
{            
	ros::init(argc, argv, "zmart_img_proc");
	ros::NodeHandle nh;
	cv::namedWindow("Line abstract", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Original image", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("Region of Interest", CV_WINDOW_AUTOSIZE);
	//cv::namedWindow("Draw Point", CV_WINDOW_AUTOSIZE);
	cv::moveWindow("Line abstract",300,50);
	cv::moveWindow("Original image",800,400);
	cv::moveWindow("Region of Interest",300,600);

	dataFile.open("Lines.txt", ios::out);
	if(!dataFile)                        
              {
                   cout<<"error open"<<endl;
                  exit(0);    
               }
               ifstream istr("Lines.txt");

               dataFile2.open("Slope.txt", ios::out);
	if(!dataFile)                        
              {
                   cout<<"error open"<<endl;
                  exit(0);    
               }
               ifstream istr2("Slope.txt");

	//cv::startWindowThread();           
	cv::createTrackbar("Canny LT: ", "Line abstract", &cannyLowThreshold, 120);
	/*cv::createTrackbar("HoughThreshold: ", "view2", &HoughThreshold, 250);
	cv::createTrackbar("srn: ", "view2", &srn, 200);
	cv::createTrackbar("stn: ", "view2", &stn, 200);*/
	cv::createTrackbar("HoughPThreshold: ", "Line abstract", &HoughPThreshold, 400);
	cv::createTrackbar("minLineLength: ", "Line abstract", &minLineLength, 400);
	cv::createTrackbar("maxLineGap: ", "Line abstract", &maxLineGap, 400);
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/img_frontend/image_undist", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("Line abstract");
	cv::destroyWindow("Original image");
	cv::destroyWindow("Region of Interest");
	//cv::destroyWindow("Draw Point");
	dataFile.close();
	dataFile2.close();

	return 0;
}

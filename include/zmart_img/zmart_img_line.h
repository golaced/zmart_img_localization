#ifndef __ZMART_IMG_LINE_H__
#define __ZMART_IMG_LINE_H__

#include <opencv2/opencv.hpp>
#include <list>
#include <vector>
#include "cmath"


using namespace cv;
using namespace std;

calss ZmartImgLine
{
public:
    double A, B, C, theta, b, d, length;
    CvPoint point1;
    CvPoint point2;

public:
    ZmartImgLine();
    ZmartImgLine(CvPoint p1, CvPoint p2);
    ZmartImgLine(double angle, double distance, CvPoint near);
    ~ZmartImgLine();

    double Get_theta();
    double Get_distacne();
    CvPoint p1();
    CvPoint p2();
    vector<CvPoint> Get_boundary();
    vector<CvPoint> Get_boundary(CvPoint p1, CvPoint p2);
};




#endif /* __ZMART_IMG_LINE_H__ */

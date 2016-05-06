# include "zmart_img/zmart_img_line.h"

#include <opencv2/opencv.hpp>

#define Pi 3.1415926

using namespace std;

ZmartImgLine::ZmartImgLine()
{
}
//不能将类成员名用作构造函数的参数名
ZmartImgLine::ZmartImgLine(CvPoint p1, CvPoint p2)
{
    point1 = p1;
    point2 = p2;

    theta = atan2 ((p1.y - p2.y), (p1,x - p2.x));//the theta of the line
    if(p1.x == p2.x)
    {
        A = 1;
        B = 0;
        C = p1.x; //shu xian, why not -p1.x;
    }
    else
    {
        A = 1.0 * (p1.y - p2.y) / (p1.x - p2.x);
        B = -1;
        C = p1.y - 1.0 * p1.x * (p1.y -p2.y) / (p1.x - p2.x);
    }

    d = abs(C) / sqrt(A*A + B*B);
    //denote the distance of point(0,0) to Ax+By+C=0;
    if (theta < 0) theta += Pi;
    length = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y)* (p1.y -p2.y));

}

ZmartImgLine::ZmartImgLine(double angle, double distance, CvPoint near) // whta is distance ???? what is near????
{
    theta = angle;
    d = distance;
    if (tan(theta) == 1)
    {
        A = 1;
        B = 0;
        if (abs(near.x - d ) < abs(near.x + d))
        {
            C = -d;
        }
        else
        {
            C = d;
        }
    }
    else
    {
        A = tan(theta);
        B = -1;
        C = d * sqrt(A*A + B*B);
        if (abs(A * near.x + B * near.y - C) < abs(A*near.x + B* near.y + C))
        {
            C = -c;
        }
    }
}

ZmartImgLine::~ZmartImgLine()
{
}

double ZmartImgLine::Get_theta()
{
    return theta;
}

double ZmartImgLine::Get_distance()
{
    return d;
}

CvPoint ZmartImgLine::p1()
{
    return point1;
}

vector<CvPoint> ZmartImgLine::Get_boundary()
{
    vector<CvPoint> points;
    points = vector<CvPoint>(2);

    //when the image is 640 * 480, so comes::
    if (B != 0)
    {
        points[0].x = 0;
        points[0].y = (int)(-C/B);
        points[1].x = 640;
        points[1].y = (int)((-C - 640*A)/B);
        //figue out the y pixel when x pixel range from 0 to 800
    }
    else
    {
        points[0].x = (int)(-C / A);
        points[0].y = 0;
        points[1].x = (int)(-C / A);
        points[1].y = 480;
    }
    return points;
    //figue out the threshold point value of line on image plane
}

vector<CvPoint> ZmartImgLine::Get_boundary(CvPoint p1, CvPoint p2)
{
    //Ax+By+C = 0; y = kx+b;
    vector<CvPoint> points;
    CvPoint min, max;
    min = CvPoint(MIN(p1.x, p2.x), MIN(p1.y, p2.y));
    max = CvPoint(MAX(p1.x, p2.x), MAX(p1.y, p2.y));

    points = vector<CvPoint>(2);
    //hen xian, get the start and end x pixel point coordinate
    if (A == 0)
    {
        points[0].x = min.x;
		points[0].y = (int)(-C / B);
		points[1].x = max.x;
		points[1].y = (int)(-C / B);
    }
    //shu xian, get the start and end y pixel point coordinate
    else if (B == 0)
    {
        points[0].x = (int)(-C / A);
		points[0].y = min.y;
		points[1].x = (int)(-C / A);
		points[1].y = max.y;
    }
    else
    {
        vector<CvPoint> lpoints;
        vector<CvPoint> cross;
        cross = vector<CvPoint>(4);
        cross[0].x = min.x;
        cross[0].y = (int)((-C - cross[0].x*A)/B);
        cross[1].x = max.x;
		cross[1].y = (int)((-C - cross[1].x*A)/B);
		cross[2].y = min.y;
		cross[2].x = (int)((-C - cross[2].y*B)/A);
		cross[3].y = max.y;
		cross[3].x = (int)((-C - cross[3].y*B)/A);
		//figure out the start and end pixel point coordinate
		if (cross[0].y >= min.y && cross[0].y <= max.y)
		{
            lpoints.push_back(cross[0]);
            //
		}
		if (cross[1].y >= min.y && cross[1].y <= max.y)
		{
			lpoints.push_back(cross[1]);
		}
		if (cross[2].x >= min.x && cross[2].x <= max.x)
		{
			lpoints.push_back(cross[2]);
		}
		if (cross[3].x >= min.x && cross[3].x <= max.x)
		{
			lpoints.push_back(cross[3]);
		}
		if(lpoints.size() < 2)
		{
            return vector<CvPoint>(0);
		}

		points[0] = lpoints[0];
		for(int i=0; i<lpoints.size(); i++)
		{
            CvPoint p = lpoints[i];
            if(!(p.x == points[0].x && p.y == points[0].y))
                points[1] = p;
		}
		// what the means?????
    }

    point1 = points[0];
    point2 = points[1];
    return points;
}






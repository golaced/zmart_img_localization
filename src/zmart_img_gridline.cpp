#include "zmart_img/zmart_img_gridline.h"

ZmartImgGridline::ZmartImgGridline(ZmartImgLine zmartimgline)
{
    //声明 对象 自动调用默认构造函数，但不初始化器对象成员
    d = zmartimgline.d;
    A = zmartimgline.A;
    B = zmartimgline.B;
    C = zmartimgline.C;
    theta = zmartimgline.theta;
    length = zmartimgline.length;
    point1 = zmartimgline.point1;
    point2 = zmartimgline.point2;
}

ZmartImgGridline::ZmartImgGridline();
{
    stability = 0.5;
}

ZmartImgGridline::~ZmartImgGridline(void)
{

}

//can not understand the following two function????
//d = |C|/sqrt(A*A+B*B) = |C|*cos theta;
void ZmartImgGridline::toPre(ZmartImgLine zmartimgline, bool dir)
{
    if (dir == HORIZONTAL) //define HORIZONTAL 0
    //this situation is horizontal line
    {
        double temp_d1, temp_d2;
        //whar does this mean?????
        if(zmartimgline.B * zmartimgline.C > 0)
            temp_d1 = -zmartimgline.d;
        else
            temp_d1 = zmartimgline.d;

        if (B*C > 0)
            temp_d2 = -d;
        else
            temp_d2 = d;

        d_to_pre = abs(temp_d1*cos(zmartimgline.theta) - temp_d2*cos(theta));
        //what's the difference between zmartimgline.theta and theta
    }
    else // k !=0
	{
		double temp_d1,temp_d2;
		if(line.A * line.C >0)
            temp_d1 = -zmartimgline.d;
		else
            temp_d1 = zmartimgline.d;

		if(A*C >0)
            temp_d2 = -d;
		else
            temp_d2 = d;

		d_to_pre = abs(temp_d1*sin(zmartimgline.theta) - temp_d2*sin(theta));
	}
}

void ZmartImgGridline::toPost(ZmartImgLine zmartimgline, bool dir)
{
    if (dir == HORIZONTAL)
    {
        double temp_d1, temp_d2;
        if(zmartimgline.B * zmartimgline.C > 0)
            temp_d1 = -zmartimgline.d;
        else
            temp_d1 = zmartimgline.d;

        if (B*C > 0)
            temp_d2 = -d;
        else
            temp_d2 = d;

        d_to_post = abs(temp_d1*cos(zmartimgline.theta) - temp_d2*(theta));
    }
    else
	{
		double temp_d1,temp_d2;
		if(line.A * line.C >0)
            temp_d1 = -line.d;
		else
            temp_d1 = line.d;

		if(A*C >0)
            temp_d2 = -d;
		else
            temp_d2 = d;

		d_to_post = abs(temp_d1*sin(zmartimgline.theta) - temp_d2*sin(theta));
	}


}

//what does this two function mean???
void ZmartImgGridline::toPreNull()
{
    d_to_pre = -1;
}

void ZmartImgGridline::toPostNull()
{
    d_to_post = -1;
}

double ZmartImgGridline::toGridLine(ZmartImgGridline zmartimgline, bool dir)
{
    if (dir == HORIZONTAL)
    {
        double temp_d1, temp_d2;
        if(zmartimgline.B * zmartimgline.C > 0)
            temp_d1 = -zmartimgline.d;
        else
            temp_d1 = zmartimgline.d;
        if(B*C >0)
            temp_d2 = -d;
		else
            temp_d2 = d;

		return abs(temp_d1*cos(zmartimgline.theta) - temp_d2*cos(theta));
    }
    else
	{
		double temp_d1,temp_d2;
		if(zmartimgline.A * zmartimgline.C >0)
            temp_d1 = -zmartimgline.d;
		else
            temp_d1 = zmartimgline.d;

		if(A*C >0)
            temp_d2 = -d;
		else
            temp_d2 = d;

		return abs(temp_d1*sin(zmartimgline.theta) - temp_d2*sin(theta));
	}
}

double ZmartImgGridline::GetToPost()
{
    return d_to_post;
}

double ZmartImgGridline::GetToPost()
{
    return d_to_pre;
}

double ZmartImgGridline::toPoint(CvPoint p)
{
    double d = abs((A*p.x + B*p.y + C) / sqrt(A*A+B*B));
    //denotes the distance of point p to the line Ax+By+C= 0
    return d;
}




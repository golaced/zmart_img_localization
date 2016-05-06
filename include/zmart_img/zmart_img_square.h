#ifndef __ZMART_IMG_SQUARE_H__
#define __ZMART_IMG_SQUARE_H__

using namespace cv;

class ZmartImgSquare
{
public:
    Point2d p1,p2,p3,p4;
    int area, id, x_pos, y_pos;
    int scale;
    double line[4][3], vertex[4][2];//vertex: 顶点
    double reliability;

public:
    ZmartImgSquare(void);
    ~ZmartImgSquare(void);
    ZmartImgSquare(Point2d px1, Point2d px2, Point2d px3, Point2d px4, int count, double re, int pos_x, int pos_y, int set_scale);

};






#endif /* __ZMART_IMG_SQUARE_H__

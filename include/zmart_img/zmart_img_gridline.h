#ifndef __ZMART_IMG_GRIDLINE_H__
#define __ZMART_IMG_GRIDLINE_H__


#include "zmart_img/zmart_img_line.h"

#define HORIZONTAL 0
#define VERTICAL 1

using namespace cv;

class ZmartImgGridline : Public ZmartImgLine
{
public:
    double threshold, d_to_pre, d_to_post;
    int world_lable, offset;
    double confidence_level, stability;

public:
    ZmartImgGridline();
    ZmartImgGridline(ZmartImgLine zmartimgline);
    ~ZmartImgeGridline(void);

    void toPre(ZmartImgLine zmartimgline, bool dir);
    void toPost(ZmartImgLine zmartimgline, bool dir);
    void toPreNull();
    void toPostNull();
    double GetToPost();
    double GetToPre();
    double toGridLine(ZmartImgGridline zmartimgline, bool dir);
    double toPoint(CvPoint p);
};





#endif /* __ZMART_IMG_GRIDLINE_H__ */

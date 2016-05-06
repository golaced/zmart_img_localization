#ifndef __ZMART_IMG_TRACKER_H__
#define __ZMART_IMG_TRACKER_H__

#include <vector>

#include "zmart_img/zmart_img_gridline.h"
#include "zmart_img/zmart_img_square.h"
#include "zmart_img/zmart_img_d_unit.h"
#include "zmart_img/zmart_img_line.h"

#include <ros/ros.h>
#include <geometry_msg/Point.h>

#include <opencv2/opencv.hpp>
#include <iostream>


#define HORIZONTAL 0
#define VERTICAL 1

using namespace cv;
using namespace std;

class ZmartImgTracker
{
public:
    int l;
    double error_x, error_y;
    double d_upper_threshold, d_lower_threshold;
    double min_horizontal_d, min_vertical_d;
    int frame_offset_horizontal, frame_offset_vertical;
    bool initialized;

    vector<int> frame_offset_h;
    vector<int> frame_offset_v;

    vector<ZmartImgSquare> zmartimgsquare;
    Mat_<double> rotation_vector, translation_vector, translation_vector_calc;
    vector<Mat_<double>> rotation_vectors, translation_vectors, translation_coordinate_;
    vector<vector <Point2d> > img_point_sets;
    vector<vector <Point3d> > obj_point_sets;
    vector<vector <Point2d> > img_point_square_sets;
    vector<vector <Point2d> > obj_point_square_sets;

    vector<Point2d> imagePoints0, imagePoints_origin, ProjectPosition, ProjectPosition_origin;

    //vector<robot_unit> robot_sets;
    Point3d coordinate_tracker, coordinate_tracker_hopped;//hop:跳跃

public:
    ZmartImgTracker(void);
    ~ZmartImgTracker(void);

    vector<ZmartImgGridline> vertical_last, horizontal_last, vertical, horizontal;



    bool Initialize(vector<ZmartImgLine> horizontal, vector<ZmartImgLine> vertical);

    void LoadGrids(vector<ZmartImgLine> horizontal_raw, vector<ZmartImgLine> vertical_raw);

    void Draw2Image(IplImage* res_im);
    //void Draw2Image(Mat res_im);
    bool IsInitialized();
    void ProcessFrame(vector<ZmartImgLine> horizontal_raw, vector<ZmartImg>)
    bool FindOffSet();
	int FindOffsetMode(vector<int> offset);


}
#endif /** __ZMART_IMG_TRACKER_H__  **/

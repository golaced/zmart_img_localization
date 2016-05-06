#ifndef __SPARSE_FLOW_H__
#define __SPARSE_FLOW_H__

class SparseFlow:Public BaseFlow
{
public:
    SparseFlow();
    ~SparseFlow();

    virtual void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    virtual void heightCallback(const sensor_msgs::RangeConstPtr& msg);

    int point_num;//
    vector<Point3i> point_vec;//
    int err_qLevel;//
    vector<Point2f> pre_points, cur_points;
    vector<uchar> status;
    vector<float> err;//
};

#endif /* __SPARSE_FLOW_H__ */

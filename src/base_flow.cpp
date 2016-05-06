#include <zmart_img/base_flow.h>


using namespace std;

BaseFlow::BaseFlow():nh("~")//BaseFlow::BaseFlow()
{
    ROS_INFO("INFO: STARTING BASE_FLOW CLASS...");
    height = 0;
}

BaseFlow::~BaseFlow()
{
    ROS_INFO("INFO: DESTROYING BASE_FLOW CLASS...");
}

Point3i BaseFlow::weng_method(vector<Point3i> vec)
{
    float min_dis = 600000;//?
    int index; //
    for (size_t i = 0; i < vec.size(); i++)
    {
        float sum = 0;
        for (size_t j = 0; j < vec.size(); j++)
        {
            if (i != j)
                sum += abs(vec[i].x - vec[j].x) + abs(vec[i].y - vec[j].y);//

        }
        if (sum < min_dis)
        {
            min_dis = sum;
            index = i;
        }
    }
    return Point3i(vec[index].x, vec[index].y, index);
}


bool comp(const Point3i &a, const Point3i &b)
{
    return a.z < b.z;//
}

Point3i BaseFlow::like_ransac(vector<Point3i> vec)
{
    Point2i center_init, center_good;
    float sum_x = 0, sum_y = 0;
    for(size_t i =0; i < vec.size(); i++)
    {
        sum_x += vec[i].x;
        sum_y += vec[i].y;
    }

    center_init.x = sum_x/vec.size();
    center_init.y = sum_y/vec.size();//the center of all the point

     sort(vec.begin(), vec.end(), comp); //upsort

     sum_x = 0; sum_y = 0;
     for (size_t i = 0; i < vec-size()/2; i++)
     {
        sum_x += vec[i].x;
        sum_y += vec[i].y;
     }

     center_init.x = sum_x/(vec.size()/2);
     center_init.y = sum_y/(vec.size()/2);//after upsort and center???

     return Point3i(center_init.x, center_init,y,0);

}













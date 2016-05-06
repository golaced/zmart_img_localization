#include <zmart_img/zmart_img_fusion.h>

int main(int argc, char * argv[])
{
    freopen("/home/exbot/catkin_ws_eclipse/src/lsd_line/attitude.txt", "w", stdout);
    ros::init(argc, argv, "zmart_img_fusion");
    ZmartImgFusion zmartimgfusion;
    ros::spin();
    return 0;
}

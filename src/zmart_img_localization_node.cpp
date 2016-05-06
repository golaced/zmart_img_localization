#include <zmart_img/zmart_img_localization.h>

int main(int argc, char *argv[])
{

    ros::init (argc, argv, "zmart_img_localization");
    ZmartImgLocalization zmartimglocalization;
    freopen("/home/exbot/catkin_ws_eclipse/src/zmart_img/out_world_imu.txt", "w", stdout) ;
    ros::spin();
    return 0;
}

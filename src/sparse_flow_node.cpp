#include <zmart_img/sparse_flow.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "SparseFlowNode");
    SparseFlow sparse_flow;
    ros::spin();
    return 0;
}

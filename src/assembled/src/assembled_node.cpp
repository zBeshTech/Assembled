#include"assembled.h"


int main(int argc, char** argv ){
    ros::init(argc, argv, "assembled_node");
    Assembled assembled;
    ros::spin();
    return 0;
}
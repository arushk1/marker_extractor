#include "marker_extractor/marker_extractor.h"
#include "ros/ros.h"

using namespace std;

int main(int argc,char**argv){
    ros::init(argc, argv, "marker_extractor");
    MarkerExtractor mex;
    int i = 0;
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

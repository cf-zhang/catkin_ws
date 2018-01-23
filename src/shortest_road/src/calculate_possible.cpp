#include "shortest_road.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "shortest_calcute");
    shortest_road::ShortestRoad srd;

    srd.calculateRoadPossibility();
    srd.distributePosition();
    ROS_INFO_STREAM("finished distribute.");
    return 0;    
}

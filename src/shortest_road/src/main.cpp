#include "shortest_road.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "shortest_calcute");
    shortest_road::ShortestRoad srd;

    srd.getShortestPaths();

    srd.writePositionFile();
    srd.writeDistanceFile();


    return 0;    
}

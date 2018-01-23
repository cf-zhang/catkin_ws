#include "shortest_road.h"


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "shortest_calcute");
    shortest_road::ShortestRoad srd(4);

    // srd.calculateRoadPossibility();
    // srd.distributePosition();


    // srd.readPossiableRoadFile();

    srd.getShortestPathsfromPathNodes();

    // srd.writePositionFile();
    srd.writeDistanceFile();
    return 0;    
}

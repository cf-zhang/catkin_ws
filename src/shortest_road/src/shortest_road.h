#ifndef _SHORTEST_ROAD_H_
#define _SHORTEST_ROAD_H_

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>

namespace shortest_road{
class TargetX{
public:
    enum TargetType
    {
        ROADWAY,
        STARTPOINT,
        RECHECKSTAGE,
        ILLEGAL
    };    
    public:
        TargetX(const float x=0.0, const float y=0.0, 
                const TargetType t=ILLEGAL):__x(x),__y(y), __type(t){}
        float getX()const {return __x;}
        float getY()const {return __y;}
        void setX(const float x){__x = x;}
        void setY(const float y){__y = y;}
        TargetType getType()const {return __type;}
        void setType(std::string name){
            if(name[0] == 'Q')__type = STARTPOINT; //1
            else if(std::string::npos != name.find("FH")) __type = RECHECKSTAGE;//2
            else __type = ROADWAY;//0
            }
        const std::string &getName()const {return __name;}
        void setName(const std::string &name){__name = name;}

    private:
        std::string __name;
        TargetType __type;
        float __x;
        float __y;
};//class TargetX

class PathNode{
    public:
        void setFrom(const TargetX &f){__from = f;}
        void setTo(const TargetX &t){__to = t;}
        void setDistance(const float dist){__distance = dist;}
        TargetX& getFrom(){return __from;}
        TargetX& getTo(){return __to;}
        float getDistance(){return __distance;}
    private:
        TargetX __from;
        TargetX __to;
        float __distance;
};



class ShortestRoad{

    public:
        ShortestRoad();
        ShortestRoad(unsigned int p);
        // ~ShortestRoad();
        bool getShortestPath(const TargetX &t1, const TargetX &t2, PathNode &pnode);
        unsigned int getShortestPaths();//return num of the shortest paths

        bool writePositionFile();
        bool writeDistanceFile();
        bool readPossiableRoadFile();
        bool writePossiableRoadFile(const unsigned int index, const unsigned int count, const unsigned int i, bool flag);
        bool readPositionFile();
        unsigned int calculateRoadPossibility();
        void distributePosition();
        unsigned int getShortestPathsfromPathNodes();

    private:
        float calculateDistance(const std::vector<geometry_msgs::PoseStamped> &path);
        float calcutePointToPointDistance(const geometry_msgs::Point &p1,const geometry_msgs::Point &p2);
        bool loadTarget(const std::string &file);
        void splits(const std::string& s, std::vector<std::string>& v, const std::string& c);
        std::string getPositionTypeName(TargetX::TargetType t);
        std::string __fileName;
        std::string __positionfile;
        std::string __distancefile;
        ros::NodeHandle __nh;
        ros::ServiceClient __client;
        ros::Publisher path_pub ;
        bool __visiabel;
        unsigned int __splitCnt;
        // std::vector<char> v;
        std::map<std::string,TargetX> __targets;//所有的目标点信息
        std::vector<PathNode> __pathNodes;
};//class ShortestRoad

}//namespace shortest_road

#endif

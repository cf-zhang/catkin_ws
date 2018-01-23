#include "shortest_road.h"
#include <navfn/MakeNavPlan.h>
#include <cmath>
#include <fstream>
#include <set>
#include <nav_msgs/Path.h>

namespace shortest_road{
ShortestRoad::ShortestRoad(){
    __visiabel = false;
    __nh.getParam("visiable",__visiabel);
    int cnt;
    __nh.getParam("splitcount",cnt);
    __splitCnt = cnt;
    if(!__nh.getParam("markerfile",__fileName) 
    || !__nh.getParam("positionfile",__positionfile)
    || !__nh.getParam("distancefile",__distancefile))
    {
        ROS_ERROR_STREAM("can not read params.");
        ros::shutdown();        
    }
    if(!loadTarget(__fileName)){
        ROS_ERROR_STREAM("can not open marker file : "<<__fileName);
        ros::shutdown();
    }

    writePositionFile();
    __client = __nh.serviceClient<navfn::MakeNavPlan>("/global_planner/make_plan");

    path_pub = __nh.advertise<nav_msgs::Path>("shortest_path", 5);
}


ShortestRoad::ShortestRoad(unsigned int p){
    __visiabel = false;
    __nh.getParam("visiable",__visiabel);
    if(!__nh.getParam("markerfile",__fileName)
    || !__nh.getParam("positionfile",__positionfile)
    || !__nh.getParam("distancefile",__distancefile))
    {
        ROS_ERROR_STREAM("can not read params.");
        ros::shutdown();        
    }
    if(!readPossiableRoadFile()){
        ROS_ERROR_STREAM("can not open marker file : "<<__fileName);
        ros::shutdown();
    }
    __splitCnt = p;
    __client = __nh.serviceClient<navfn::MakeNavPlan>("/global_planner/make_plan");

    path_pub = __nh.advertise<nav_msgs::Path>("shortest_path", 5);
}





//计算两点间的距离和
float ShortestRoad::calcutePointToPointDistance(const geometry_msgs::Point &p1,const geometry_msgs::Point &p2){
    float x2 = (p1.x - p2.x)*(p1.x - p2.x);
    float y2 = (p1.y - p2.y)*(p1.y - p2.y);
    return sqrt(x2+y2);
}
//计算得到的最短路径中所有点之间的距离和
float ShortestRoad::calculateDistance(const std::vector<geometry_msgs::PoseStamped> &path){
    float dist;
    std::vector<geometry_msgs::PoseStamped>::const_iterator iter;
    geometry_msgs::Point p1,p2;
    for(iter = path.begin(); iter != path.end()-1; iter++)
    {
        p1 = iter->pose.position;
        p2 = (iter+1)->pose.position;
        dist += calcutePointToPointDistance(p1, p2);
    }
    return dist;
}


//想make_path服务发起请求，获取最短路径之后，进行长度计算。
bool ShortestRoad::getShortestPath(const TargetX &t1, const TargetX &t2, PathNode &pnode){

    navfn::MakeNavPlan mkPathSrv;
    mkPathSrv.request.start.pose.position.x = t1.getX();
    mkPathSrv.request.start.pose.position.y = t1.getY();
    mkPathSrv.request.start.header.stamp = ros::Time::now();
    mkPathSrv.request.start.header.frame_id = "map";
    mkPathSrv.request.goal.pose.position.x = t2.getX();
    mkPathSrv.request.goal.pose.position.y = t2.getY();
    mkPathSrv.request.goal.header.stamp = ros::Time::now();
    mkPathSrv.request.goal.header.frame_id = "map";
    if(!__client.call(mkPathSrv)){
        ROS_ERROR_STREAM("request the make plan server fail.");
        return false;
    }

    if(__visiabel)
    {
        nav_msgs::GetPlan PathSrv;
        PathSrv.response.plan.header.frame_id="map";
        PathSrv.response.plan.poses = mkPathSrv.response.path;
        path_pub.publish(PathSrv.response.plan);
        ros::Duration(5).sleep();
    }
    if(mkPathSrv.response.path.size() < 2)
    {
        ROS_ERROR_STREAM("have no path from "<<t1.getName()<<" to "<<t2.getName());
        return false;        
    }
    float dist = calculateDistance(mkPathSrv.response.path);
    pnode.setFrom(t1);
    pnode.setTo(t2);
    pnode.setDistance(dist);
    return true;
}

//获取所有目标点之间的最短距离，存放于__pathNodes中
unsigned int ShortestRoad::getShortestPaths(){
    std::map<std::string,TargetX> tmp = __targets;
    std::map<std::string,TargetX>::iterator iteri,iterj;
    TargetX t;//获取节点内容
    PathNode pnode;
    for(iteri = tmp.begin(); iteri != tmp.end(); iteri = tmp.begin()){
        t = iteri->second;
        tmp.erase(iteri);
        for(iterj = tmp.begin(); iterj != tmp.end(); iterj++){
            if(getShortestPath(t, iterj->second, pnode)){
                __pathNodes.push_back(pnode);    
                // std::cout<<pnode.getFrom().getName()<<"--->"<<pnode.getTo().getName()<<"  length :"<<pnode.getDistance()<<std::endl;
            }
            else{
                // ROS_WARN_STREAM("can not get the distance between "<<pnode.getFrom().getName()<<" and "<<pnode.getTo().getName()<std::endl);
            }
        }
    }
    return __pathNodes.size();
}

std::string ShortestRoad::getPositionTypeName(TargetX::TargetType t){
    switch (t)
    {
        case TargetX::ROADWAY:
            return "roadway";
        case TargetX::RECHECKSTAGE:
            return "recheckstage";
        case TargetX::STARTPOINT:
            return "startpoint";
        default:
            return "";
    }
}

//将各个目标点的坐标写入文件中
bool ShortestRoad::writePositionFile(){
    std::ofstream ofile(__positionfile.c_str());
    if(!ofile.is_open()){
        ROS_ERROR_STREAM("can not open file "<<__positionfile);
        return false;
    }
    ofile<<"position_type,position_code,position_x,position_y"<<std::endl;
    // std::map<std::string,TargetX> __targets;
    std::map<std::string,TargetX>::iterator iter;
    std::string ptype,pcode;
    float px,py;
    for(iter = __targets.begin(); iter != __targets.end(); iter++)
    {
        
        ptype = getPositionTypeName(iter->second.getType());
        pcode = iter->second.getName();
        px = iter->second.getX();
        py = iter->second.getY();
        ofile<<ptype<<","<<pcode<<","<<px<<","<<py<<std::endl;
    }
    ofile.close();
    return true;
}

//将各个目标点的坐标读入内存中
bool ShortestRoad::readPositionFile(){
    std::ifstream ifile(__positionfile.c_str());
    if(!ifile.is_open()){
        ROS_ERROR_STREAM("can not open file "<<__positionfile);
        return false;
    }
    TargetX target;
    std::string line;
    std::vector<std::string> v;
    getline(ifile, line);//read head content

    while(getline(ifile, line)){
        splits(line, v, ",");
        target.setName(v[1]);
        target.setX(atof(v[2].c_str()));
        target.setY(atof(v[3].c_str()));
        target.setType(v[1]);
        __targets.insert(std::map<std::string,TargetX>::value_type(v[1], target));
        v.clear();
    }
    ifile.close();
    return true;
}

unsigned int ShortestRoad::calculateRoadPossibility(){
    std::map<std::string,TargetX> tmp = __targets;
    std::map<std::string,TargetX>::iterator iteri,iterj;
    PathNode pnode;
    for(iteri = tmp.begin(); iteri != tmp.end(); iteri = tmp.begin()){

        tmp.erase(iteri);
        for(iterj = tmp.begin(); iterj != tmp.end(); iterj++){
            pnode.setFrom(iteri->second);
            pnode.setTo(iterj->second);
            pnode.setDistance(0.0);
            __pathNodes.push_back(pnode);
        }
    }
    return __pathNodes.size();
}
void ShortestRoad::distributePosition(){
    unsigned int total = __pathNodes.size();
    unsigned int contentSplit = __splitCnt==0? 1 : total / __splitCnt;
    for(unsigned int i = 0; i < __splitCnt; i++){
        writePossiableRoadFile(contentSplit*i, contentSplit, i, i==__splitCnt-1);
    }

}



bool ShortestRoad::writeDistanceFile(){
    std::ofstream ofile(__distancefile.c_str());
    if(!ofile.is_open()){
        ROS_ERROR_STREAM("can not open file "<<__distancefile);
        return false;
    }
    ofile<<"from_position,to_position,distance"<<std::endl;
    // std::vector<PathNode> __pathNodes;
    std::vector<PathNode>::iterator iter;
    for(iter = __pathNodes.begin(); iter != __pathNodes.end(); iter++)
    {
        ofile<<iter->getFrom().getName()<<","
             <<iter->getTo().getName()<<","
             <<iter->getDistance()<<std::endl;
    }
    ofile.close();
    return true;
}


bool ShortestRoad::writePossiableRoadFile(const unsigned int index, const unsigned int count, const unsigned int i, bool flag){
    std::string subfix[] = {"1","2","3","4","5","6","7","8","9"};
    std::string distancefile = __distancefile + subfix[i];
    std::ofstream ofile(distancefile.c_str());
    if(!ofile.is_open()){
        ROS_ERROR_STREAM("can not open file "<<__distancefile);
        return false;
    }
    ofile<<"from_position,x,y, to_position,x,y"<<std::endl;
    // std::vector<PathNode> __pathNodes;
    // std::vector<PathNode>::iterator iter;
    unsigned int max = index+count;
    for(int i = index; i < (flag ? __pathNodes.size() : max); i++)
    {
        ofile<<__pathNodes[i].getFrom().getName()<<","
             <<__pathNodes[i].getFrom().getX()<<","
             <<__pathNodes[i].getFrom().getY()<<","
             <<__pathNodes[i].getTo().getName()<<","
             <<__pathNodes[i].getTo().getX()<<","
             <<__pathNodes[i].getTo().getY()<<std::endl;
            
    }
    ofile.close();
    return true;
}


//将各个目标点的坐标读入内存中
bool ShortestRoad::readPossiableRoadFile(){
    std::ifstream ifile(__fileName.c_str());
    if(!ifile.is_open()){
        ROS_ERROR_STREAM("can not open file "<<__positionfile);
        return false;
    }
    TargetX target1,target2;
    PathNode pnode;
    std::string line;
    std::vector<std::string> v;
    getline(ifile, line);//read head content
    std::string tname;
    while(getline(ifile, line)){
        splits(line, v, ",");
        target1.setName(v[0]);
        target1.setX(atof(v[1].c_str()));
        target1.setY(atof(v[2].c_str()));
        target1.setType(v[0]);
        // __targets.insert(std::map<std::string,TargetX>::value_type(v[0], target));
        target2.setName(v[3]);
        target2.setX(atof(v[4].c_str()));
        target2.setY(atof(v[5].c_str()));
        target2.setType(v[3]);
        // __targets.insert(std::map<std::string,TargetX>::value_type(v[3], target));
        v.clear();
        // std::vector<PathNode> __pathNodes;
        pnode.setFrom(target1);
        pnode.setTo(target2);
        pnode.setDistance(0.0);
        __pathNodes.push_back(pnode);
    }
    ifile.close();
    return true;
}
unsigned int ShortestRoad::getShortestPathsfromPathNodes(){
   unsigned int len = __pathNodes.size();
   for(unsigned int i = 0; i < len; i++){

        if(getShortestPath(__pathNodes[i].getFrom(), __pathNodes[i].getTo(), __pathNodes[i])){
            if(i % 100 == 0){
                std::cout<<"has been dealed with "<<i<<std::endl;
            }
        // std::cout<<pnode.getFrom().getName()<<"--->"<<pnode.getTo().getName()<<"  length :"<<pnode.getDistance()<<std::endl;
        }
        else{
            // ROS_WARN_STREAM("can not get the distance between "<<pnode.getFrom().getName()<<" and "<<pnode.getTo().getName()<std::endl);
        } 
   } 
}













const std::string targetNames[]= {
"B00A-0A-1","B00A-0A-2","B00A-0B-1","B00A-0B-2","B00A-0C-1",
"B00A-0C-2","B00A-0D-1","B00A-0D-2","B00A-0E-1","B00A-0E-2",
"B00A-0F-1","B00A-0F-2","B00A-0G-1","B00A-0G-2","B00A-0H-1",
"B00A-0H-2","B00A-0I-1","B00A-0I-2","B00A-0J-1","B00A-0J-2",
"B00A-0K-1","B00A-0K-2","B00A-0L-1","B00A-0L-2","B00A-0M-1",
"B00A-0M-2","B00A-0N-1","B00A-0N-2","B00A-0O-1","B00A-0O-2",
"B00A-0P-1","B00A-0P-2","B00A-0Q-1","B00A-0Q-2","B00A-0R-1",
"B00A-0R-2","B00A-0S-1","B00A-0S-2","B00A-0T-1","B00A-0T-2",

"B00B-0A-1","B00B-0A-2","B00B-0B-1","B00B-0B-2","B00B-0C-1",
"B00B-0C-2","B00B-0D-1","B00B-0D-2","B00B-0E-1","B00B-0E-2",
"B00B-0F-1","B00B-0F-2","B00B-0G-1","B00B-0G-2","B00B-0H-1",
"B00B-0H-2","B00B-0I-1","B00B-0I-2","B00B-0J-1","B00B-0J-2",
"B00B-0K-1","B00B-0K-2","B00B-0L-1","B00B-0L-2","B00B-0M-1",
"B00B-0M-2","B00B-0N-1","B00B-0N-2","B00B-0O-1","B00B-0O-2",
"B00B-0P-1","B00B-0P-2","B00B-0Q-1","B00B-0Q-2","B00B-0R-1",
"B00B-0R-2","B00B-0S-1","B00B-0S-2","B00B-0T-1","B00B-0T-2",
"B00B-0U-1","B00B-0U-2","B00B-0V-1","B00B-0V-2",

"B00C-0A-1","B00C-0A-2","B00C-0B-1","B00C-0B-2","B00C-0C-1",
"B00C-0C-2","B00C-0D-1","B00C-0D-2","B00C-0E-1","B00C-0E-2",
"B00C-0F-1","B00C-0F-2","B00C-0G-1","B00C-0G-2","B00C-0H-1",
"B00C-0H-2","B00C-0I-1","B00C-0I-2","B00C-0J-1","B00C-0J-2",
"B00C-0K-1","B00C-0K-2","B00C-0L-1","B00C-0L-2","B00C-0M-1",
"B00C-0M-2","B00C-0N-1","B00C-0N-2","B00C-0O-1","B00C-0O-2",
"B00C-0P-1","B00C-0P-2","B00C-0Q-1","B00C-0Q-2","B00C-0R-1",
"B00C-0R-2","B00C-0S-1","B00C-0S-2","B00C-0T-1","B00C-0T-2",
"B00C-0U-1","B00C-0U-2","B00C-0V-1","B00C-0V-2",

"B00D-0A-1","B00D-0A-2","B00D-0B-1","B00D-0B-2","B00D-0C-1",
"B00D-0C-2","B00D-0D-1","B00D-0D-2","B00D-0E-1","B00D-0E-2",
"B00D-0F-1","B00D-0F-2","B00D-0G-1","B00D-0G-2","B00D-0H-1",
"B00D-0H-2","B00D-0I-1","B00D-0I-2","B00D-0J-1","B00D-0J-2",
"B00D-0K-1","B00D-0K-2","B00D-0L-1","B00D-0L-2","B00D-0M-1",
"B00D-0M-2","B00D-0N-1","B00D-0N-2","B00D-0O-1","B00D-0O-2",
"B00D-0P-1","B00D-0P-2","B00D-0Q-1","B00D-0Q-2","B00D-0R-1",
"B00D-0R-2","B00D-0S-1","B00D-0S-2","B00D-0T-1","B00D-0T-2",
"B00D-0U-1","B00D-0U-2","B00D-0V-1","B00D-0V-2",

"B00E-0A-1","B00E-0A-2","B00E-0B-1","B00E-0B-2","B00E-0C-1",
"B00E-0C-2","B00E-0D-1","B00E-0D-2","B00E-0E-1","B00E-0E-2",
"B00E-0F-1","B00E-0F-2","B00E-0G-1","B00E-0G-2","B00E-0H-1",
"B00E-0H-2","B00E-0I-1","B00E-0I-2","B00E-0J-1","B00E-0J-2",
"B00E-0K-1","B00E-0K-2","B00E-0L-1","B00E-0L-2","B00E-0M-1",
"B00E-0M-2","B00E-0N-1","B00E-0N-2","B00E-0O-1","B00E-0O-2",
"B00E-0P-1","B00E-0P-2","B00E-0Q-1","B00E-0Q-2","B00E-0R-1",
"B00E-0R-2","B00E-0S-1","B00E-0S-2","B00E-0T-1","B00E-0T-2",
"B00E-0U-1","B00E-0U-2","B00E-0V-1","B00E-0V-2",

"B00F-0A-1","B00F-0A-2","B00F-0B-1","B00F-0B-2","B00F-0C-1",
"B00F-0C-2","B00F-0D-1","B00F-0D-2","B00F-0E-1","B00F-0E-2",
"B00F-0F-1","B00F-0F-2","B00F-0G-1","B00F-0G-2","B00F-0H-1",
"B00F-0H-2","B00F-0I-1","B00F-0I-2","B00F-0J-1","B00F-0J-2",
"B00F-0K-1","B00F-0K-2","B00F-0L-1","B00F-0L-2","B00F-0M-1",
"B00F-0M-2","B00F-0N-1","B00F-0N-2","B00F-0O-1","B00F-0O-2",
"B00F-0P-1","B00F-0P-2","B00F-0Q-1","B00F-0Q-2","B00F-0R-1",
"B00F-0R-2","B00F-0S-1","B00F-0S-2","B00F-0T-1","B00F-0T-2",

"B00G-0A-1","B00G-0A-2","B00G-0B-1","B00G-0B-2","B00G-0C-1",
"B00G-0C-2","B00G-0D-1","B00G-0D-2","B00G-0E-1","B00G-0E-2",
"B00G-0F-1","B00G-0F-2","B00G-0G-1","B00G-0G-2","B00G-0H-1",
"B00G-0H-2","B00G-0I-1","B00G-0I-2","B00G-0J-1","B00G-0J-2",
"B00G-0K-1","B00G-0K-2","B00G-0L-1","B00G-0L-2","B00G-0M-1",
"B00G-0M-2","B00G-0N-1","B00G-0N-2","B00G-0O-1","B00G-0O-2",
"B00G-0P-1","B00G-0P-2","B00G-0Q-1","B00G-0Q-2","B00G-0R-1",
"B00G-0R-2","B00G-0S-1","B00G-0S-2","B00G-0T-1","B00G-0T-2",

"B00H-0A-1","B00H-0A-2","B00H-0B-1","B00H-0B-2","B00H-0E-1","B00H-0E-2",
"B00H-0F-1","B00H-0F-2","B00H-0G-1","B00H-0G-2","B00H-0H-1",
"B00H-0H-2","B00H-0I-1","B00H-0I-2","B00H-0J-1","B00H-0J-2",
"B00H-0K-1","B00H-0K-2","B00H-0L-1","B00H-0L-2","B00H-0M-1",
"B00H-0M-2","B00H-0N-1","B00H-0N-2","B00H-0O-1",
"B00H-0O-2","B00H-0P-1","B00H-0P-2",

"B00I-0A-1","B00I-0A-2","B00I-0B-1","B00I-0B-2","B00I-0C-1",
"B00I-0C-2","B00I-0D-1","B00I-0D-2","B00I-0E-1","B00I-0E-2",
"B00I-0F-1","B00I-0F-2","B00I-0G-1","B00I-0G-2","B00I-0H-1",
"B00I-0H-2","B00I-0I-1","B00I-0I-2","B00I-0J-1","B00I-0J-2",
"B00I-0K-1","B00I-0K-2","B00I-0L-1","B00I-0L-2","B00I-0M-1",
"B00I-0M-2","B00I-0N-1","B00I-0N-2","B00I-0O-1","B00I-0O-2",
"B00I-0P-1","B00I-0P-2","B00I-0Q-1","B00I-0Q-2","B00I-0R-1",
"B00I-0R-2","B00I-0S-1","B00I-0S-2","B00I-0T-1","B00I-0T-2",
"B00I-0U-1","B00I-0U-2","B00I-0V-1","B00I-0V-2",

"B00J-0A-1","B00J-0A-2","B00J-0B-1","B00J-0B-2","B00J-0C-1",
"B00J-0C-2","B00J-0D-1","B00J-0D-2","B00J-0E-1","B00J-0E-2",
"B00J-0F-1","B00J-0F-2","B00J-0G-1","B00J-0G-2","B00J-0H-1",
"B00J-0H-2","B00J-0I-1","B00J-0I-2","B00J-0J-1","B00J-0J-2",
"B00J-0K-1","B00J-0K-2","B00J-0L-1","B00J-0L-2","B00J-0M-1",
"B00J-0M-2","B00J-0N-1","B00J-0N-2","B00J-0O-1","B00J-0O-2",
"B00J-0P-1","B00J-0P-2","B00J-0Q-1","B00J-0Q-2","B00J-0R-1",
"B00J-0R-2","B00J-0S-1","B00J-0S-2","B00J-0T-1","B00J-0T-2",
"B00J-0U-1","B00J-0U-2","B00J-0V-1","B00J-0V-2",

"B00K-0A-1","B00K-0A-2","B00K-0B-1","B00K-0B-2","B00K-0C-1",
"B00K-0C-2","B00K-0D-1","B00K-0D-2","B00K-0E-1","B00K-0E-2",
"B00K-0F-1","B00K-0F-2","B00K-0G-1","B00K-0G-2","B00K-0H-1",
"B00K-0H-2","B00K-0I-1","B00K-0I-2","B00K-0J-1","B00K-0J-2",
"B00K-0K-1","B00K-0K-2","B00K-0L-1","B00K-0L-2","B00K-0M-1",
"B00K-0M-2","B00K-0N-1","B00K-0N-2","B00K-0O-1","B00K-0O-2",
"B00K-0P-1","B00K-0P-2",

"B00L-0A-1","B00L-0A-2","B00L-0B-1","B00L-0B-2","B00L-0C-1",
"B00L-0C-2","B00L-0D-1","B00L-0D-2","B00L-0E-1","B00L-0E-2",
"B00L-0F-1","B00L-0F-2","B00L-0G-1","B00L-0G-2","B00L-0H-1",
"B00L-0H-2","B00L-0I-1","B00L-0I-2","B00L-0J-1","B00L-0J-2",
"B00L-0K-1","B00L-0K-2","B00L-0L-1","B00L-0L-2","B00L-0M-1",
"B00L-0M-2","B00L-0N-1","B00L-0N-2","B00L-0O-1","B00L-0O-2",
"B00L-0P-1","B00L-0P-2","B00L-0Q-1","B00L-0Q-2","B00L-0R-1",
"B00L-0R-2","B00L-0S-1","B00L-0S-2","B00L-0T-1","B00L-0T-2",

"B00M-0A-1","B00M-0A-2","B00M-0B-1","B00M-0B-2","B00M-0C-1",
"B00M-0C-2","B00M-0D-1","B00M-0D-2","B00M-0E-1","B00M-0E-2",
"B00M-0F-1","B00M-0F-2","B00M-0G-1","B00M-0G-2","B00M-0H-1",
"B00M-0H-2","B00M-0I-1","B00M-0I-2","B00M-0J-1","B00M-0J-2",
"B00M-0K-1","B00M-0K-2",

"B00N-0A-1","B00N-0A-2","B00N-0B-1","B00N-0B-2","B00N-0C-1",
"B00N-0C-2","B00N-0D-1","B00N-0D-2","B00N-0E-1","B00N-0E-2",
"B00N-0F-1","B00N-0F-2","B00N-0G-1","B00N-0G-2","B00N-0H-1",
"B00N-0H-2","B00N-0I-1","B00N-0I-2","B00N-0J-1","B00N-0J-2",
"B00N-0K-1","B00N-0K-2","B00N-0L-1","B00N-0L-2",

"B00O-0A-1","B00O-0A-2","B00O-0B-1","B00O-0B-2","B00O-0C-1",
"B00O-0C-2","B00O-0D-1","B00O-0D-2","B00O-0E-1","B00O-0E-2",
"B00O-0F-1","B00O-0F-2","B00O-0G-1","B00O-0G-2","B00O-0H-1",
"B00O-0H-2","B00O-0I-1","B00O-0I-2","B00O-0J-1","B00O-0J-2",

"B00P-0A-1","B00P-0A-2","B00P-0B-1","B00P-0B-2","B00P-0C-1",
"B00P-0C-2","B00P-0D-1","B00P-0D-2","B00P-0E-1","B00P-0E-2",

"B00Q-0C-1",
"B00Q-0C-2","B00Q-0D-1","B00Q-0D-2","B00Q-0E-1","B00Q-0E-2",
"B00Q-0F-1","B00Q-0F-2",

"B001-0B-1","B001-0B-2","B001-0D-1","B001-0D-2",
"B001-0F-1","B001-0F-2","B001-0H-1",
"B001-0H-2","B001-0J-1","B001-0J-2","B001-0L-1","B001-0L-2",
"B001-0N-1","B001-0N-2","B001-0P-1","B001-0P-2","B001-0R-1","B001-0R-2",

"B002-0A-1","B002-0A-2","B002-0B-1","B002-0B-2","B002-0C-1",
"B002-0C-2","B002-0D-1","B002-0D-2","B002-0E-1","B002-0E-2",
"B002-0F-1","B002-0F-2","B002-0G-1","B002-0G-2","B002-0H-1",
"B002-0H-2","B002-0I-1","B002-0I-2","B002-0J-1","B002-0J-2",
"B002-0K-1","B002-0K-2","B002-0L-1","B002-0L-2","B002-0M-1",
"B002-0M-2","B002-0N-1","B002-0N-2","B002-0O-1","B002-0O-2",
"B002-0P-1","B002-0P-2","B002-0Q-1","B002-0Q-2","B002-0R-1",
"B002-0R-2","B002-0S-1","B002-0S-2","B002-0T-1","B002-0T-2",
"B002-0U-1","B002-0U-2","B002-0V-1","B002-0V-2","B002-0W-1",
"B002-0W-2","B002-0X-1","B002-0X-2","B002-0Y-1","B002-0Y-2",
"B002-0Z-1","B002-0Z-2","B002-01-1","B002-01-2","B002-02-1",
"B002-02-2",

"B003-0B-1","B003-0B-2","B003-0D-1","B003-0D-2",
"B003-0F-1","B003-0F-2","B003-0G-1","B003-0G-2","B003-0H-1",
"B003-0H-2","B003-0I-1","B003-0I-2","B003-0J-1","B003-0J-2",
"B003-0K-1","B003-0K-2","B003-0L-1","B003-0L-2","B003-0M-1",
"B003-0M-2","B003-0N-1","B003-0N-2","B003-0O-1","B003-0O-2",
"B003-0P-1","B003-0P-2","B003-0Q-1","B003-0Q-2","B003-0R-1",
"B003-0R-2","B003-0S-1","B003-0S-2","B003-0T-1","B003-0T-2",
"B003-0U-1","B003-0U-2","B003-0V-1","B003-0V-2","B003-0W-1",
"B003-0W-2","B003-0X-1","B003-0X-2","B003-0Y-1","B003-0Y-2",
"B003-0Z-1","B003-0Z-2",

"Q001","Q002","Q003","Q004","Q005","Q006",

"A2FH001","A2FH002","A2FH003","A2FH004","A2FH005","A2FH006","A2FH007",
"A2FH008","A2FH009","A2FH0010","A2FH0011","A2FH0012","A2FH0013","A2FH0014",
"A2FH0015","A2FH0016","A2FH0017","A2FH0018","A2FH0019","A2FH020","A2FH021",
"A2FH022","A2FH023","A2FH024","A2FH025","A2FH026","A2FH027","A2FH028",
"A2FH029","A2FH0030"

};


//加载文件中所有的目标点到map中，获取名字和坐标对应关系
bool ShortestRoad::loadTarget(const std::string &file){
	// open the file  
	std::ifstream inFile( file.c_str(), std::ios_base::in);
	if (!inFile.is_open()){
		ROS_ERROR("file read failed");
		// ROS_ERROR("%s",filename.c_str());
		return false;
	}

    std::string line;

    TargetX target;
    std::string targetname;
    std::vector<std::string> v;
    while(getline(inFile, line)){  
        // std::cout<<t++<<"line: "<<line<<std::endl;  
        if('#' == line[0]){
            targetname = line.substr(1);
            targetname = targetNames[atoi(targetname.c_str())];
            target.setName(targetname);
            getline(inFile, line);
            splits(line, v, "\t ");
            
            target.setX(atof(v.back().c_str()));
            getline(inFile, line);
            splits(line, v, "\t ");

            target.setY(atof(v.back().c_str()));
            target.setType(targetname);
            std::cout<<target.getName()<<" "<<target.getX()<<" "<<target.getY() <<" "<< target.getType()<<std::endl;
            __targets.insert(std::map<std::string,TargetX>::value_type(targetname, target));
            // std::cout<<"code: " <<targetname<<" x: "<<target.getX()<<" y: "<<target.getY()<<" type :"<<target.getType()<<std::endl;
        }
    }  
    inFile.close();
    return true;
}

//以c中包含的字符对s进行切分
void ShortestRoad::splits(const std::string& s, std::vector<std::string>& v, const std::string& c){
    std::string::size_type pos1, pos2;
    std::set<char> splits;
    int i = 0;
    while(c[i])
        splits.insert(c[i++]);

    pos1 = 0;
    i = 0;
    while(s[i]){
        if(splits.count(s[i]) ){
            pos2 = i;
            if(pos2-pos1 > 0)
                v.push_back(s.substr(pos1, pos2-pos1));
            pos1 = pos2+1;
        }
        i++;
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}
}//namespace shortest_road


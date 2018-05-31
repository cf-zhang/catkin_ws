#include "launchall.h"
#include <ros/ros.h>
#include <unistd.h>
LaunchAll::LaunchAll()
{
  stop = false;
}
void LaunchAll::startLaunch(){
  std::string launchFile = "/home/cfzhang/xcode/catkin_qt/src/system.launch";
  launchFile.insert(0, "roslaunch ");
  ROS_INFO_STREAM("system launch file :"<<launchFile);
  char cmdresult[1024];
  FILE *pp = popen(launchFile.c_str(), "r"); //建立管道
  pclose(pp);
}
void LaunchAll::stopLaunch(){
  std::string launchFile = "/home/cfzhang/xcode/catkin_qt/src/killprocess.sh roslaunch";
  launchFile.insert(0, "sh ");
  ROS_INFO_STREAM("system launch file :"<<launchFile);
  char cmdresult[1024];
  FILE *pp = popen(launchFile.c_str(), "r"); //建立管道
  pclose(pp);
  ROS_INFO("system launch is shutdown successfully.");
}

void LaunchAll::run()
{
  bool firststart = true;
  ros::V_string nodes;
  ros::master::getNodes(nodes);

  for(ros::V_string::iterator it = nodes.begin(); it != nodes.end(); it++)
  {
    if(*it == "/NodeA" || *it == "/NodeB"){
      firststart = false;
      break;
    }
  }
  if(firststart){
    ROS_INFO("is first start.");
    startLaunch();
  }
  while(!stop){
    sleep(1);
  }
  stopLaunch();
}

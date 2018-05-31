#include "ros/ros.h"
#include "testgui/gui.h"
class NodeB{
public:
  NodeB():value(0),step(1){
     service = nh.advertiseService("/BServer", &NodeB::bServerCall, this);
     checkService = nh.advertiseService("/BServerSelfCheck", &NodeB::bServerSelfCheckCall, this);
     nh.param<int>("/NodeB/step", step, 1);
     nh.param<int>("/NodeB/base", value, 0);
  }
  bool bServerCall(testgui::gui::Request  &req,
                  testgui::gui::Response &res)
  {
    value += step;
    ROS_INFO("value: %ld", value);
    ROS_ERROR("return value: %ld", value);
    res.value = value;
    return true;
  }

  bool bServerSelfCheckCall(testgui::gui::Request  &req,
                            testgui::gui::Response &res){

    ROS_INFO("in node B self check callback.");
    res.value = 1;
    return true;
  }
private:
  ros::NodeHandle nh;
  ros::ServiceServer service;
  ros::ServiceServer checkService;
  int value;
  int step;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "NodeB");


  NodeB nodeB;

  ROS_INFO("NodeB is Ready to server.");
  ros::spin();

  return 0;
}

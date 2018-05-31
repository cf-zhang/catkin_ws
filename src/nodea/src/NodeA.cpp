#include "ros/ros.h"
#include "testgui/gui.h"
class NodeA{
public:
  NodeA():value(0),step(1){
     service = nh.advertiseService("/AServer", &NodeA::aServerCall, this);
     checkService = nh.advertiseService("/AServerSelfCheck", &NodeA::aServerSelfCheckCall, this);

//     std::string s;
     nh.param<int>("/NodeA/step", step, 1);
     nh.param<int>("/NodeA/base", value, 0);
  }
  bool aServerCall(testgui::gui::Request  &req,
                  testgui::gui::Response &res)
  {
    value += step;
    ROS_INFO("value: %d", value);
    ROS_ERROR("return value: %d", value);
    res.value = value;
    return true;
  }

  bool aServerSelfCheckCall(testgui::gui::Request  &req,
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
  ros::init(argc, argv, "NodeA");


  NodeA nodeA;

  ROS_INFO("NodeA is Ready to server.");
  ros::spin();

  return 0;
}

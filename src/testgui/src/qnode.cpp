/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <unistd.h>
#include "../include/testgui/qnode.hpp"
#include "testgui/gui.h"
#include "launchall.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace testgui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    la.setStop();
    sleep(5);
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"testgui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
  // Add your ros communications here.
  toAClient = n.serviceClient<testgui::gui>("/AServer");
  toBClient = n.serviceClient<testgui::gui>("/BServer");
  checkAClient = n.serviceClient<testgui::gui>("/AServerSelfCheck");
  checkBClient = n.serviceClient<testgui::gui>("/BServerSelfCheck");
	start();
	return true;
}


void QNode::run() {


  la.start();

  if(ros::service::waitForService("/AServerSelfCheck")){

    testgui::gui checkASrv;
    if(checkAClient.call(checkASrv))
      ROS_INFO("Node A self check is ok.");
  }
  if(ros::service::waitForService("/BServerSelfCheck")){

    testgui::gui checkBSrv;
    if(checkBClient.call(checkBSrv))
      ROS_INFO("Node B self check is ok.");
  }
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
    msg.data = ss.str();
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool QNode::toAClientCall(int *value)
{
  testgui::gui srv;
  bool rt = toAClient.call(srv);
  if(rt)
  {
    *value = srv.response.value;
    return true;
  }

  return false;
}
bool QNode::toBClientCall(int *value)
{
  testgui::gui srv;
  bool rt = toBClient.call(srv);
  if(rt)
  {
    *value = srv.response.value;
    return true;
  }

  return false;
}
void QNode::log( const LogLevel &level, const std::string &msg) {

	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}

}

}  // namespace testgui

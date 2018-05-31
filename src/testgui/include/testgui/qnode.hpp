/**
 * @file /include/testgui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef testgui_QNODE_HPP_
#define testgui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include "../../src/launchall.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace testgui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	void log( const LogLevel &level, const std::string &msg);
  bool toAClientCall(int *value);
  bool toBClientCall(int *value);
Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

  ros::ServiceClient toAClient;
  ros::ServiceClient toBClient;
  ros::ServiceClient checkAClient;
  ros::ServiceClient checkBClient;
  LaunchAll la;
};

}  // namespace testgui

#endif /* testgui_QNODE_HPP_ */

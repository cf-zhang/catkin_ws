#include <QApplication>

#include <ros/ros.h>
#include "widget.h"
class ButtonCtlApp : public QApplication
{
public:
  ros::NodeHandlePtr nh_;

  ButtonCtlApp(int& argc, char** argv)
    : QApplication(argc, argv)
  {
    ros::init(argc, argv, "log_panel", ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle);
  }

  int exec()
  {
    Widget w;
    w.show();
    // w.showFullScreen();
    // w.showMaximized();
    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  ButtonCtlApp app(argc, argv);
  return app.exec();
}

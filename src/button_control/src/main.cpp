#include <QApplication>

#include <ros/ros.h>
#include "widget.h"
#include "logindlg.h"

class ButtonCtlApp : public QApplication
{
public:
  ros::NodeHandlePtr nh_;

  ButtonCtlApp(int& argc, char** argv)
    : QApplication(argc, argv)
  {
    ros::init(argc, argv, "button_control", ros::init_options::NoSigintHandler);
    nh_.reset(new ros::NodeHandle);
  }

  int exec()
  {
    Widget w;
    //w.show();
    w.showFullScreen();
    // w.showMaximized();
    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  ButtonCtlApp app(argc, argv);
  LoginDlg login;
  if(login.exec()==QDialog::Accepted)
  {
    return app.exec();
  }
  else
    return 1;
}


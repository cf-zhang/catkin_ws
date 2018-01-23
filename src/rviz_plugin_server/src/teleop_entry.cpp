#include <stdio.h>

#include <ros/console.h>

#include <QPainter>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QHeaderView>
#include <QFileDialog>
#include <QString>
#include <geometry_msgs/Twist.h>
#include "teleop_entry.h"
#include "map_server/GetMaps.h"
#include <lift_msgs/GetMapID.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>

namespace rviz_plugin_server
{

TeleopEntry::TeleopEntry( QWidget* parent )
  : rviz::Panel( parent )
{
  // ROS_INFO_STREAM("entry constructor is called. ");
  //添加widget，然后设置显示
  hbt_layout = new QVBoxLayout;

  tabWidget = new QTabWidget();
  //添加map面板
  tmp = TeleopMapPanel::getMapPanel();
  tabWidget->addTab(tmp, "map ctrl");
  //添加目标点面板
  tgp = TeleopGoalPanel::getGoalPanel();
  tabWidget->addTab(tgp, "goal ctrl");

  //添加robot信息面板
  model = TeleopModel::getModel();
  tabWidget->addTab(model, "robot info");

  logpanel = new log_panel::Widget();
  tabWidget->addTab(logpanel, "log view");
  //设置显示
  hbt_layout->addWidget(tabWidget);
  setLayout( hbt_layout );

  //设置定时器
  int idd = startTimer(100);//设置100ms的定时器
  // ROS_ERROR_STREAM("in entry: "<<idd);
  serviceForRobots = nh_.advertiseService("/robot_info", &TeleopEntry::robotinfoCallback, this);
  
}



//描述：作为服务/robot_info的回调函数，用于处理goalpanel中机器人注册入口，
//     marker中机器人显示初始化入口
//参数：
//    @req：请求内容
//    @res：反馈内容
//返回值：true：成功; false:失败
bool TeleopEntry::robotinfoCallback(rviz_plugin_server::robotInfo::Request &req,
                     rviz_plugin_server::robotInfo::Response &res)
{

  //目标点面板中robot信息的注册入口
  tgp->robotInfoRegister(req.roboname);

  //interactive marker处理接口，用于机器人初始显示姿态的设置
  model->createRobotModel(req.roboname, req.pose, 
                          req.rssi, req.power, 
                          req.location);


  // //其他处理逻辑,然后返回
  // //
  // //
  // //
  return true;
}

//同步目标控制面板和状态信息面板的选中机器人目标一致
//2S刷新goalBox内容， 200ms刷新robotModel View，0.5S同步两个面板内容一次
void TeleopEntry::syncCtrlAndInfoPanel(const int timeCount)
{
  static int curIndex = 0;
  std::vector<std::string> robotsRemoved;
  robotsRemoved.clear();
  if(timeCount % tgp->TimeForRobotInfoRefresh<<1 == 0)
  {//刷新goalBox中在线的robot名字
    tgp->robotInfoRefresh(robotsRemoved);
    // ROS_INFO_STREAM("2 S ");
  }
  
  if(timeCount % model->TimeForRobotViewRefresh == 0)
  {//刷新当前显示的robots状态
    model->robotViewRefresh(robotsRemoved);
    // ROS_INFO_STREAM("200 mS ");
  }

  if(timeCount % 500)//0.5S刷新同步一次
  {
    if(model->isActive)
    {
      // tabWidget->setTabEnabled(2, true);
      tabWidget->setCurrentWidget(model);
      curIndex = tgp->robotBox->findText(QString::fromStdString(model->curRobot));
      tgp->robotBox->setCurrentIndex(curIndex);
      model->isActive = false;
    }
    else
    { 
      if(curIndex != tgp->robotBox->currentIndex())
      {
        model->curRobot = tgp->robotBox->currentText().toStdString();
        curIndex = tgp->robotBox->findText(QString::fromStdString(model->curRobot));
        model->setRobotStatus(model->curRobot);
      }
    }

  }

}

void TeleopEntry::syncMapserverAndMapPanel(const int timeCount)
{

  if(0 == timeCount % tmp->TimeForMapRefresh && false == tmp->isMapServerLaunched)
  {
    tmp->updateMap();
    // ROS_INFO_STREAM("2 S ");
  }
  if((0 == timeCount % (tmp->TimeForMapRefresh<<5)) && (true == tmp->isMapServerLaunched))
  {//多次开关面板会 coredump。
    // ROS_INFO_STREAM("check for map server status ");
    tmp->updateMap();
  }

}


//描述：定时器事件函数，负责清理下线机器人的超时逻辑，机器人下拉框中条目的更新操作
//参数：@t： 自动传参，可以根据t->timerID()来区分具体是哪一个定时器触发了事件
//    @res：反馈内容
//返回值：无
void TeleopEntry::timerEvent(QTimerEvent *t)//定时器时间
{
  // ROS_INFO_STREAM("timer event");
  static int timeCount = 0;

  //获取成功前2S刷新频率   获取成功后10S检查频率
  syncMapserverAndMapPanel(timeCount);
  //2S刷新goalBox内容， 200ms刷新robotModel View，0.5S同步两个面板内容一次
  syncCtrlAndInfoPanel(timeCount);

  timeCount += 100;
}

void TeleopEntry::closeEvent(QCloseEvent *event)
{
  ROS_INFO_STREAM("close event is called. ");
  std::cout<<"close";
}
//rviz中每次打开面板的时候，都会调用构造函数，但是点击×的时候并不会调用析构，
//会在最后关闭rviz的时候才会去调用析构，如果我们频繁的开关了多次这个插件面板
//那么会在最后出现coredump错误，已经尝试过重写closeEvent虚函数进行解决，但是
//并没有生效。
TeleopEntry::~TeleopEntry()
{
  // if(NULL != tmp)  delete tmp;
  // if(NULL != tgp)  delete tgp;
  // if(NULL != model) delete model;
  // if(NULL != tabWidget) delete tabWidget;
  // if(NULL != hbt_layout) delete hbt_layout;
  // ROS_INFO_STREAM("entry destructor is called. ");
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopEntry::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
}

// Load all configuration data for this panel from the given Config object.
void TeleopEntry::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_server::TeleopEntry,rviz::Panel )
// END_TUTORIAL

#include "teleop_model.h"

#include <QString>

// #include <QInputDialog>
#include <QMessageBox>
#include <rviz/visualization_manager.h>
#include <rviz/visualization_frame.h>
#include <rviz/visualizer_app.h>
namespace rviz_plugin_server
{

TeleopModel* TeleopModel::getModel()
{
  static TeleopModel *model = NULL;
  if(model == NULL)
  {
    model = new TeleopModel(0);
  }
  return model;
}

TeleopModel::TeleopModel(QWidget* parent)
  : rviz::Panel( parent ),TimeForRobotViewRefresh(200)
{
  server = new interactive_markers::InteractiveMarkerServer("robotViews","",false);

  //添加三个按钮，然后水平排列
  hbt_layout = new QHBoxLayout;
  followBtn = new QPushButton("Follow", this);
  backBtn = new QPushButton("Back", this);
  chargeBtn = new QPushButton("Charge", this);

  hbt_layout->addWidget(followBtn);  
  hbt_layout->addWidget(backBtn);  
  hbt_layout->addWidget(chargeBtn);  

  //添加power，并进行设置
  power_view = new QLabel(this);
  power_view->setText("Power : ");
  power = new QLabel(this);
  power->setFrameStyle(QFrame::Panel | QFrame::Sunken);  
  power->setText("0%");  
  power->setAlignment( Qt::AlignCenter); 


  //添加robot name，并进行设置
  rbtname_view = new QLabel(this);
  rbtname_view->setText("Robot : ");
  rbtname = new QLabel(this);
  rbtname->setFrameStyle(QFrame::Panel | QFrame::Sunken);  
  rbtname->setText("");  
  rbtname->setAlignment( Qt::AlignCenter); 


  //添加rssi，并进行设置
  rssi_view = new QLabel(this);
  rssi_view->setText("RSSI : ");
  rssi = new QLabel(this);
  rssi->setFrameStyle(QFrame::Panel | QFrame::Sunken);  
  rssi->setText("0");  
  rssi->setAlignment( Qt::AlignCenter); 

  //添加location，并进行设置
  location_view = new QLabel(this);
  location_view->setText("Location : ");
  location = new QLabel(this);
  location->setFrameStyle(QFrame::Panel | QFrame::Sunken);  
  location->setText("(X: 0.0, Y: 0.0, Z: 0.0)");  
  location->setAlignment( Qt::AlignCenter); 

  pLayout = new QGridLayout();

  // power label 第0行，第0列开始，占1行1列
  pLayout->addWidget(rbtname_view, 0, 0, 1, 1);
  // power content 第0行，第1列开始，占1行1列
  pLayout->addWidget(rbtname, 0, 1, 1, 1);
 
  // rssi label 第0行，第0列开始，占1行1列
  pLayout->addWidget(power_view, 1, 0, 1, 1);
  // rssi content 第0行，第1列开始，占1行1列
  pLayout->addWidget(power, 1, 1, 1, 1);

  // location label 第0行，第0列开始，占1行1列
  pLayout->addWidget(rssi_view, 2, 0, 1, 1);
  // location content 第0行，第1列开始，占1行1列
  pLayout->addWidget(rssi, 2, 1, 1, 1);

  // location label 第0行，第0列开始，占1行1列
  pLayout->addWidget(location_view, 3, 0, 1, 1);
  // location content 第0行，第1列开始，占1行1列
  pLayout->addWidget(location, 3, 1, 1, 1);
  
  //设置两列空间的占比
  pLayout->setColumnStretch(0, 2);
  pLayout->setColumnStretch(1, 3);
  // 设置水平间距
  pLayout->setHorizontalSpacing(10);
  // 设置垂直间距
  pLayout->setVerticalSpacing(10);
  // 设置外间距
  pLayout->setContentsMargins(10, 10, 10, 10);
  // setLayout(pLayout);


  //布局管理，将上面的控件进行垂直布局
  v_layout = new QVBoxLayout;
  v_layout->addLayout( hbt_layout );
  v_layout->addLayout( pLayout );
  setLayout( v_layout );


  //设置信号和槽
  connect( followBtn, SIGNAL( pressed () ), this, SLOT( followBtnClicked() ));
  connect( backBtn,   SIGNAL( pressed () ), this, SLOT( backBtnClicked() ));
  connect( chargeBtn, SIGNAL( pressed () ), this, SLOT( chargeBtnClicked() ));

  planCli = nh_.serviceClient<nav_msgs::GetPlan>("planplan");
  path_pub = nh_.advertise<nav_msgs::Path>("testpath", 1000);

  menu_handler.insert( "single-poin-set", &processFeedback );
  menu_handler.insert( "multi-poin-set", &processFeedback );
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

  tool.onInitialize();
}



//根据curRobot的内容，赋值这个机器人的客户端信息，并返回操作结果成功 true，失败 false
bool TeleopModel::getCtlClient(ros::ServiceClient& client)
{
  if(curRobot.empty())
  {
    ROS_INFO_STREAM("please choose a robot for control.");
      // ROS_INFO("add click button is pressed!");
    QMessageBox msgBox;
    msgBox.setText("please choose a robot for control.");
    QPushButton *ok = msgBox.addButton(QMessageBox::Ok);
    int ret = msgBox.exec();
    return false;
  }
  
  // std::map<std::string,ros::ServiceClient> robotControl;//机器人状态容器
  std::map<std::string,ros::ServiceClient>::iterator it;
  it = robotControl.find(curRobot);
  // ROS_INFO_STREAM(curRobot);
  if(it == robotControl.end())
  {//不存在该机器人的控制信息
    ROS_INFO_STREAM("invalid input, please choose a robot for control,again.");
      // ROS_INFO("add click button is pressed!");
    QMessageBox msgBox;
    msgBox.setText("invalid input, please choose a robot for control,again.");
    QPushButton *ok = msgBox.addButton(QMessageBox::Ok);
    int ret = msgBox.exec();
    return false;    
  }

  client = it->second;
  return true;
}

//跟随按钮的回调函数
void TeleopModel::followBtnClicked()
{
  ros::ServiceClient client;
  if(!getCtlClient(client))
  {
    return ;
  }

  //确认存在该robot的控制客户端信息，进行发送控制命令ID
  rviz_plugin_server::robotCtl srv;
  srv.request.cmdID = rviz_plugin_server::robotCtlRequest::FOLLOWEMODE;
  if(client.call(srv))
  {
    ROS_INFO_STREAM("follow cmd for "+ curRobot +" request success.");
  }
  else
  {
    ROS_INFO_STREAM("follow cmd for "+ curRobot +" request fail.");
  }

}
//返航按钮的回调函数
void TeleopModel::backBtnClicked()
{
  ros::ServiceClient client;
  if(!getCtlClient(client))
  {
    return ;
  }

  //确认存在该robot的控制客户端信息，进行发送控制命令ID
  rviz_plugin_server::robotCtl srv;
  srv.request.cmdID = rviz_plugin_server::robotCtlRequest::BACKMODE;
  if(client.call(srv))
  {
    ROS_INFO_STREAM("back cmd for "+ curRobot +" request success.");
  }
  else
  {
    ROS_INFO_STREAM("back cmd for "+ curRobot +" request fail.");
  }

}
//充电按钮的回调函数
void TeleopModel::chargeBtnClicked()
{
  ros::ServiceClient client;
  if(!getCtlClient(client))
  {
    return ;
  }

  //确认存在该robot的控制客户端信息，进行发送控制命令ID
  rviz_plugin_server::robotCtl srv;
  srv.request.cmdID = rviz_plugin_server::robotCtlRequest::CHARGEMODE;
  if(client.call(srv))
  {
    ROS_INFO_STREAM("charge cmd for "+ curRobot +" request success.");
  }
  else
  {
    ROS_INFO_STREAM("charge cmd for "+ curRobot +" request fail.");
  }

}
//marker模型的创建以及样式颜色等的设置
Marker TeleopModel::makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 1.0;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}
//交互marker的控制器模式的创建和设置
InteractiveMarkerControl& TeleopModel::makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.interaction_mode = InteractiveMarkerControl::BUTTON;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

//rviz中使用鼠标点击模型时的回调函数，feedback中会携带点击事件的相关信息
void TeleopModel::processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback )
{
  TeleopModel *model = TeleopModel::getModel();
  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK)
  {
    // ROS_INFO_STREAM("in processFeedback: "<< feedback->marker_name);
    
    model->curRobot = feedback->marker_name;
    model->setRobotStatus(model->curRobot);
    model->isActive = true;//当有机器人被鼠标点击时，激活状态面板
  }

  if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    ROS_INFO_STREAM(": menu item " << feedback->menu_entry_id << " clicked" );
    model->curRobot = feedback->marker_name;
    switch (feedback->menu_entry_id)
    {
      case 1:
          // rviz::VisualizerApp::getStaticVisualizationFrame()->onToolbarActionTriggered(&getModel()->tool);
          getModel()->tool.setIsSingle(true);
          getModel()->tool.setIsMultiContinue(true);
        break;
      case 2:
          getModel()->tool.setIsMultiContinue(true);
        // while(1)
        // {
        //   if(getModel()->tool.getIsMultiContinue())
        //     rviz::VisualizerApp::getStaticVisualizationFrame()->onToolbarActionTriggered(&getModel()->tool);
        // }
        // rviz::VisualizerApp::getStaticVisualizationFrame()->onToolbarActionTriggered(&getModel()->tool);
        break;

      default:
        break;
        /* Default Code */
    }
    
  }


}  




//设置robot的状态信息，包括信号强度rssi， 电池电量power，位置坐标location
//根据传入的机器人名称来从robotStatus中找到该机器人，并获取各项指标值以后刷新到界面上
//param @name 需要设置的机器人名称
void TeleopModel::setRobotStatus(const std::string& name)//, const int rssi_, const int power_, const geometry_msgs::Point &location_
{
  int rssi_, power_;
  geometry_msgs::Point location_;//中间变量
  std::map<std::string,RobotStatus>::iterator it;
  it = robotStatus.find(name);
  if(it == robotStatus.end())
  {
    rssi_ = 0;
    power_ = 0;
    location_.x = location_.y = location_.z = 0;
  }
  else
  {
    rssi_ = it->second.getRSSI();
    power_ = it->second.getPower(); 
    location_ = it->second.getLocation();
  }

  char buffer[30];
  //name
  rbtname->setText(name.c_str());
  //power
  sprintf(buffer, "%d", power_);
  power->setText(buffer);
  //rssi
  sprintf(buffer, "%d", rssi_);
  rssi->setText(buffer);
  //location
  sprintf(buffer, "( %.2f, %.2f, %.2f )", location_.x, location_.y, location_.z);
  location->setText(buffer);
}

//检查当前传入的机器人是否已经存在了没有,如果已经存在，则需要更新这个机器人的位置信息
//如果该机器人尚未存在，那么就在server中插入一个机器人以及这个机器人的pose信息
//param： @name 机器人名称
//param： @pose 机器人当前姿态
void TeleopModel::robotModelCheckForCreate(const std::string& name,  const geometry_msgs::Pose& pose )
{
  InteractiveMarker int_marker;

  if(server->get(name, int_marker))
  {//已经存在，更新pose
    // ROS_INFO_STREAM("robot is alive already pose.x "<<pose.position.x);
    int_marker.pose = pose;
  }
  else
  {//未存在，插入新robot
    int_marker.header.frame_id = "map";
    int_marker.pose = pose;
    int_marker.scale = 2;
    int_marker.name = name;
    int_marker.description = name;
    // insert a box
    makeBoxControl(int_marker);
  }
  server->insert(int_marker);
  
  server->setCallback(int_marker.name, &TeleopModel::processFeedback);
  menu_handler.apply( *server, int_marker.name );  //menue
  server->applyChanges();
}

//检查当前传入的机器人是否已经存在状态信息了没有,如果已经存在，则需要更新这个机器人的状态信息
//如果该机器人尚未存在，那么就在robotStatus中插入一个机器人以及这个机器人的状态信息
//param： @name 机器人名称
//param： @rssi 机器人当前信号强度
//param： @power 机器人当前电量信息
//param： @location 机器人当前坐标信息
void TeleopModel::robotStatusCheckForCreate(const std::string &name, int rssi, int power, const geometry_msgs::Point &location)
{
  // std::map<std::string,RobotStatus> robotStatus;
  std::map<std::string,RobotStatus>::iterator it;
  it = robotStatus.find(name);
  if(it != robotStatus.end())
  {
    it->second.setRSSI(rssi);
    it->second.setPower(power);
    it->second.setLocation(location);
  }
  else
  {
    robotStatus.insert(std::map<std::string,RobotStatus>::value_type(name, RobotStatus(location, power, rssi)));
  }
  
}
//检查当前传入的机器人是否已经存在控制信息了没有,如果已经存在，则不进行操作，
//如果该机器人尚未存在，那么就在robotControl中插入一个机器人以及这个机器人的控制句柄
//param： @name 机器人名称
void TeleopModel::robotControlCheckForCreate(const std::string &name)
{
  // std::map<std::string,ros::ServiceClient> robotControl;//机器人状态容器
  std::map<std::string,ros::ServiceClient>::iterator it;
  it = robotControl.find(name);
  
  if(it == robotControl.end())
  {//如果不存在，就插入一条
    std::string str = "/"+name+"/robotControl";
    ros::ServiceClient client = nh_.serviceClient<rviz_plugin_server::robotCtl>(str);
    robotControl.insert(std::map<std::string,ros::ServiceClient>::value_type(name, client));
    
  }
}

//在rviz中创建一个机器人实体，作为总入口，分别调用各个分部进行创建或者更新
//如果该机器人尚未存在，那么就在robotStatus中插入一个机器人以及这个机器人的状态信息
//param： @pose 机器人当前姿态
//param： @name 机器人名称
//param： @rssi 机器人当前信号强度
//param： @power 机器人当前电量信息
//param： @location 机器人当前坐标信息
void TeleopModel::createRobotModel( const std::string &name,  const geometry_msgs::Pose& pose ,
                                    int rssi, int power, const geometry_msgs::Point &location)
{
  //机器人模型检查，存在，进行更新，如果不存在就进行创建
  robotModelCheckForCreate(name, pose);

  //robot status
  robotStatusCheckForCreate(name, rssi, power, location);

  // robot control
  robotControlCheckForCreate(name);
}



void TeleopModel::pathPlan()
{
  // gtpsrv.request.goal.pose.position.x = getModel()->tool.getFlagX();
  // gtpsrv.request.goal.pose.position.y = getModel()->tool.getFlagY();
  // planCli.call(gtpsrv);
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = getModel()->tool.getFlagX();
  pose.pose.position.y = getModel()->tool.getFlagY();
  flagPoses.push_back(pose);

}
    //将规划了的路径发出
void TeleopModel::pathPublish()
{
  nav_msgs::Path msg;
  msg.header.frame_id="map";

  //copy the plan into a message to send out
  msg.poses.resize(flagPoses.size());

  for(unsigned int i = 0; i < flagPoses.size(); ++i){
    msg.poses[i] = flagPoses[i];
  }
  path_pub.publish(msg);
}
//设置模型的路径，首先如果是单一模式，那么就设置模型位置为起点
//如果是多选择模式，则设置上一次选择的位置为起点
//
void TeleopModel::setModelPath()
{

  geometry_msgs::PoseStamped pose;
  if(getModel()->tool.getIsMultiContinue())
  {
    if(flagPoses.size() == 0)
    {
      pose.pose.position=robotStatus.find(curRobot)->second.getLocation();
      flagPoses.push_back(pose);
    }

    rviz::VisualizerApp::getStaticVisualizationFrame()->onToolbarActionTriggered(&getModel()->tool);
    if(getModel()->tool.getIsSingle())
    {
      getModel()->tool.setIsSingle(false);
      getModel()->tool.setIsMultiContinue(false);
    }
  }
  else
  {
    if(flagPoses.size() != 1)
      std::vector<geometry_msgs::PoseStamped>().swap(flagPoses);
  }

  if(getModel()->tool.getFlagX())
  {
    //获取起始点并进行规划路径
    pathPlan();
    //将规划了的路径发出
    pathPublish();
    getModel()->tool.setFlagX(0.0);
  }

}
//刷新界面上所有的机器人模型的显示，删除掉已经掉线了的机器人模型
//param： @robotRemoved 需要删除掉的下线机器人集合
void TeleopModel::robotViewRefresh(const std::vector<std::string> &robotsRemoved)
{
  int i;
  for(i = 0; i < robotsRemoved.size(); i++)
  {
    ROS_INFO_STREAM("in robotViewrefresh "<< robotsRemoved[i]);
    server->erase(robotsRemoved[i]);
    robotStatus.erase(robotsRemoved[i]);
  }
  server->applyChanges();

  setModelPath();

}

TeleopModel::~TeleopModel()
{
  delete server;

  delete pLayout;
  delete hbt_layout;
  delete v_layout;

  delete followBtn;
  delete backBtn;
  delete chargeBtn;

  delete rbtname_view;
  delete rbtname;
  delete power_view;
  delete power;
  delete rssi_view;
  delete rssi;
  delete location_view;
  delete location;
}
}
// int main(int argc, char *argv[])
// {

//   ros::init(argc, argv, "robotModel");

//   TeleopModel *pmodel = new TeleopModel();
//   tf::Vector3 position(0,0,0);
//   pmodel->createRobotModel("robot1", position);

//   ros::spin();
//   ROS_INFO("Exiting.."); 
//   return 0;
// }
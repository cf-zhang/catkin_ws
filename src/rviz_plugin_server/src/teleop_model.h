
#ifndef TELEOP_MODEL_H
#define TELEOP_MODEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif


#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <rviz_plugin_server/robotCtl.h>
#include <nav_msgs/GetPlan.h>
#include "plant_flag_tool.h"
using namespace visualization_msgs;
namespace rviz_plugin_server
{
//robotstatus 作为机器人状态记录的数据类型
//location power rssi等
class RobotStatus{
  public:
    RobotStatus(const geometry_msgs::Point &location, int power, int rssi)
    {
      this->power = power;
      this->rssi = rssi;
      this->location = location;
    }


    inline geometry_msgs::Point getLocation()
    {
      return location;
    }
    inline void setLocation(geometry_msgs::Point location)
    {
      this->location = location;
    }

    inline int getPower()
    {
      return power;
    }
    inline void setPower(int power)
    {
      this->power = power;
    }  

    inline int getRSSI()
    {
      return rssi;
    }
    inline void setRSSI(int RSSI)
    {
      this->rssi = RSSI;
    }  
  private:
    geometry_msgs::Point location;//机器人坐标
    int power;//机器人电量信息，0～100
    int rssi;//机器人信号强度
};

class TeleopModel: public rviz::Panel
{
  Q_OBJECT
  public:
    //单例模式接口
    static TeleopModel* getModel();
    bool isActive;//状态面板是否所当前状态
    std::string curRobot;//当前选中的机器人名字
    const int TimeForRobotViewRefresh;//用于控制刷新时间
    interactive_markers::InteractiveMarkerServer *server;
    interactive_markers::MenuHandler menu_handler;

    ~TeleopModel();

    //在rviz中创建一个机器人实体，作为总入口，分别调用各个分部进行创建或者更新
    //如果该机器人尚未存在，那么就在robotStatus中插入一个机器人以及这个机器人的状态信息
    //param： @pose 机器人当前姿态
    //param： @name 机器人名称
    //param： @rssi 机器人当前信号强度
    //param： @power 机器人当前电量信息
    //param： @location 机器人当前坐标信息
    void createRobotModel( const std::string &name,  const geometry_msgs::Pose& pose,
                            int rssi, int power, const geometry_msgs::Point &location);
    
    //刷新界面上所有的机器人模型的显示，删除掉已经掉线了的机器人模型
    //param： @robotRemoved 需要删除掉的下线机器人集合                            
    void robotViewRefresh(const std::vector<std::string> &robots);//定时器事件触发

    //设置robot的状态信息，包括信号强度rssi， 电池电量power，位置坐标location
    //根据传入的机器人名称来从robotStatus中找到该机器人，并获取各项指标值以后刷新到界面上
    //param @name 需要设置的机器人名称    
    void setRobotStatus(const std::string& name);

  protected:
  protected Q_SLOTS:
    //三个按钮所对应的槽处理
    void followBtnClicked();//跟随按钮的回调函数
    void backBtnClicked();//返航按钮的回调函数
    void chargeBtnClicked();//充电按钮的回调函数

  private:
  TeleopModel & operator=(const TeleopModel &){};
  TeleopModel(const TeleopModel &):TimeForRobotViewRefresh(200){};
  //按钮
  QPushButton *followBtn;
  QPushButton *backBtn;
  QPushButton *chargeBtn;
  
  //标签
  QLabel      *rbtname_view;
  QLabel      *rbtname;
  QLabel      *power_view;
  QLabel      *power;
  QLabel      *rssi_view;
  QLabel      *rssi;
  QLabel      *location_view;
  QLabel      *location;

  //布局器
  QGridLayout *pLayout;
  QHBoxLayout *hbt_layout;
  QVBoxLayout *v_layout;

  std::map<std::string,RobotStatus> robotStatus;//机器人状态容器
  ros::NodeHandle nh_;
  ros::ServiceClient  planCli ;
  ros::Publisher path_pub;
  nav_msgs::GetPlan gtpsrv;
  std::map<std::string,ros::ServiceClient> robotControl;//机器人状态容器
  rviz_plugin_server::PlantFlagTool tool;
  std::vector<geometry_msgs::PoseStamped>  flagPoses;
  TeleopModel( QWidget* parent = 0 );
  //marker模型的创建以及样式颜色等的设置
  Marker makeBox( InteractiveMarker &msg );
  //交互marker的控制器模式的创建和设置
  InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg );

  //rviz中使用鼠标点击模型时的回调函数，feedback中会携带点击事件的相关信息
  static void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback );
  
  bool getCtlClient(ros::ServiceClient& client);

  //检查当前传入的机器人是否已经存在了没有,如果已经存在，则需要更新这个机器人的位置信息
  //如果该机器人尚未存在，那么就在server中插入一个机器人以及这个机器人的pose信息
  //param： @name 机器人名称
  //param： @pose 机器人当前姿态
  void robotModelCheckForCreate(const std::string &name,  const geometry_msgs::Pose& pose );

  //检查当前传入的机器人是否已经存在状态信息了没有,如果已经存在，则需要更新这个机器人的状态信息
  //如果该机器人尚未存在，那么就在robotStatus中插入一个机器人以及这个机器人的状态信息
  //param： @name 机器人名称
  //param： @rssi 机器人当前信号强度
  //param： @power 机器人当前电量信息
  //param： @location 机器人当前坐标信息
  void robotStatusCheckForCreate(const std::string &name, int rssi, int power, const geometry_msgs::Point &location);
  
  //检查当前传入的机器人是否已经存在控制信息了没有,如果已经存在，则不进行操作，
  //如果该机器人尚未存在，那么就在robotControl中插入一个机器人以及这个机器人的控制句柄
  //param： @name 机器人名称  
  void robotControlCheckForCreate(const std::string &name);


void pathPlan();

    //将规划了的路径发出
void pathPublish();

void setModelPath();
};
}
#endif // TELEOP_PANEL_H

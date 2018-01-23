
#ifndef TELEOP_GOALPANEL_H
#define TELEOP_GOALPANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QTableWidget>
#include <QTableWidgetItem> 
#include <QStandardItemModel>
#include <QPushButton>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int64.h>

#include "rviz_plugin_server/OptGoal.h"
#include "map_server/GetMapId.h"
namespace rviz_plugin_server
{
class QRMsgsValue{
  public:
    QRMsgsValue(geometry_msgs::PoseStamped pose, bool isMarker)
    {
      this->pose = pose;
      this->isMarker = isMarker;
    }
    QRMsgsValue(geometry_msgs::PoseStamped pose, bool isMarker, int mapId)
    {
      this->pose = pose;
      this->isMarker = isMarker;
      this->mapId = mapId;
    } 
    inline geometry_msgs::PoseStamped getPose() const {return pose;}
    inline void setPose(geometry_msgs::PoseStamped pose)
    {
      this->pose = pose;
    }

    inline bool getIsMarker() const {return isMarker;}
    inline void setIsMarker(bool isMarker)
    {
      this->isMarker = isMarker;
    }

    inline int getMapId() const {return mapId;}
    inline void setMapId(int mapId)
    {
      this->mapId = mapId;
    }  
  private:
    geometry_msgs::PoseStamped pose;
    int mapId;
    bool isMarker;
};

class RobotInfoValue{
  public:

    RobotInfoValue(ros::Publisher posePub, ros::Publisher mbgoalPub, int timeout)
    {
      this->posePub   = posePub;
      this->mbgoalPub = mbgoalPub;
      this->timeout   = timeout;
    }

    inline ros::Publisher getPosePub() const {return posePub;}
    inline void setPosePub(ros::Publisher posePub)
    {
      this->posePub = posePub;
    }  

    inline ros::Publisher getMBGoalPub() const {return mbgoalPub;}
    inline void setMBGoalPub(ros::Publisher mbgoalPub)
    {
      this->mbgoalPub = mbgoalPub;
    }

    inline int getTimeOut() const {return timeout;}
    inline void setTimeOut(int timeout)
    {
      this->timeout = timeout;
    }

  private:
    ros::Publisher posePub; //set_goal
    ros::Publisher mbgoalPub;//move_base_simple/goal  
    int timeout;
    
};



  
typedef std::map<std::string, QRMsgsValue> QRMsgs;
typedef std::map<std::string, std::vector<double>  > OrignContent;
#define ROWS 4
#define COLS 4
class TeleopGoalPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  QComboBox  *robotBox;
  static TeleopGoalPanel* getGoalPanel();
  std::map<std::string, RobotInfoValue> robotInfos;
  const int TimeForRobotInfoRefresh;//用于刷新box内容超时，单位是 S

  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  
  ~TeleopGoalPanel();
  bool robotInfoRegister(std::string robotName);
  void robotInfoRefresh(std::vector<std::string> &robots);//定时器时间
  
  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:


  // Here we declare some internal slots.
protected Q_SLOTS:
  //三个按钮所对应的槽处理
  void applyBtnClicked();
  void addFileBtnClicked();
  void addClickBtnClicked();
  void delBtnClicked();
  void saveBtnClicked();
  void editConfirmFun(QStandardItem* item);
  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  

  // Then we finish up with protected member variables.
protected:
  TeleopGoalPanel( QWidget* parent = 0 );
  QHBoxLayout* hbox_layout;
  QHBoxLayout* hbt_layout;
  QHBoxLayout* h_layout;
  QVBoxLayout* v_layout;
  QTableView *tv;
  QStandardItemModel  *model;

  QPushButton *applybt;
  QPushButton *addbt;
  QPushButton *delbt;
  QPushButton *savebt;
  // The current name of the output topic.
  QString output_topic_;

  // The ROS publisher for the command velocity.
  ros::Publisher velocity_publisher_;




  private:
    TeleopGoalPanel & operator=(const TeleopGoalPanel &){};
    TeleopGoalPanel(const TeleopGoalPanel &):TimeForRobotInfoRefresh(1000){};   
    int inx;
    int curMap;
    bool isWaitForClick;
    
    std::vector<std::string> _beforeEdit;
    // QPushButton *qsave;
  // The ROS node handle.
    ros::NodeHandle nh_;
    
    ros::ServiceClient clientGetMapId_;
    ros::Subscriber nav_click_sub;
    ros::Subscriber mapId_sub_;

    QRMsgs marker_pose_;
    OrignContent content;
    
    //从文件中读取目标点信息，会将所需要信息暂时保存到marker_pose_中
    //filename： 文件绝对路径
    //返回值：    成功：读取到的目标点个数
    //           失败：-1
    int readQRTable(std::string filename, bool isMarker);         
    void goalClickCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void mapIdSubCallback(const std_msgs::Int64::ConstPtr& msg);
    void refreshQRTable();
    //将鼠标点击获取到的四元数转换成为矩阵
    std::map<std::string, std::vector<double> >::value_type convertQuat2Matrix(std::string text, geometry_msgs::PoseStamped &msg);

};

} // end namespace rviz_plugin_server

#endif 

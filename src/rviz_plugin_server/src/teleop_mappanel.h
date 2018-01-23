
#ifndef TELEOP_MAPPANEL_H
#define TELEOP_MAPPANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QTableWidgetItem> 
#include <QStandardItemModel>
#include <QPushButton>


namespace rviz_plugin_server
{

class TeleopMapPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  const int TimeForMapRefresh;
  bool isMapServerLaunched;
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  static TeleopMapPanel* getMapPanel();
  
  ~TeleopMapPanel();
  //获取mapserver端的地图信息，并显示到rviz表格中
  void updateMap();


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
  void addBtnClicked();
  void delBtnClicked();
  // updateTopic() reads the topic name from the QLineEdit and calls
  // setTopic() with the result.
  

  // Then we finish up with protected member variables.
protected:
  TeleopMapPanel( QWidget* parent = 0 );
  QHBoxLayout* hbt_layout;
  QHBoxLayout* h_layout;
  QVBoxLayout* v_layout;
  QTableView *tv;
  QStandardItemModel  *model;

  QPushButton *applybt;
  QPushButton *addbt;
  QPushButton *delbt;
  // The current name of the output topic.
  QString output_topic_;

  // The ROS publisher for the command velocity.
  ros::Publisher velocity_publisher_;

  // The ROS node handle.
  ros::NodeHandle nh_;
  ros::ServiceClient client_g;
  ros::ServiceClient client_sw;


  private:
    TeleopMapPanel & operator=(const TeleopMapPanel &){};
    TeleopMapPanel(const TeleopMapPanel &):TimeForMapRefresh(2000){};  
    int inx;


};

} // end namespace rviz_plugin_server

#endif // TELEOP_PANEL_H

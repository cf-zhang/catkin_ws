
#ifndef TELEOP_ENTRY_H
#define TELEOP_ENTRY_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif
#include "rviz_plugin_server/robotInfo.h"
#include "log_panel/widget.h"
#include "teleop_model.h"
#include "teleop_mappanel.h"
#include "teleop_goalpanel.h"
#include <QTableWidget>
#include <QTableWidgetItem> 
#include <QStandardItemModel>
#include <QPushButton>
#include <QTabWidget>
#include <QTimerEvent>
#include <QCloseEvent>
namespace rviz_plugin_server
{

class TeleopEntry: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  

  TeleopEntry( QWidget* parent = 0 );
  ~TeleopEntry();
  void timerEvent(QTimerEvent *);
  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

  // Next come a couple of public Qt slots.
public Q_SLOTS:


  // Here we declare some internal slots.
protected Q_SLOTS:

protected:
  bool robotinfoCallback(rviz_plugin_server::robotInfo::Request &req,
                     rviz_plugin_server::robotInfo::Response &res);
  void closeEvent(QCloseEvent *event);
private:
    TeleopEntry & operator=(const TeleopEntry &){};
    TeleopEntry(const TeleopEntry &){};
    QVBoxLayout* hbt_layout;
    QTabWidget *tabWidget;
    TeleopMapPanel *tmp;
    TeleopGoalPanel *tgp;
    TeleopModel     *model;
    log_panel::Widget *logpanel;
    ros::NodeHandle nh_;
    ros::ServiceServer serviceForRobots;

    void syncCtrlAndInfoPanel(const int timeCount);
    void syncMapserverAndMapPanel(const int timeCount);
};

} // end namespace rviz_plugin_server

#endif // TELEOP_PANEL_H

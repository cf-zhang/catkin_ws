
#include <QtSql>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include "dbmanager.h"


class RobotLog
{
  public:
    RobotLog();
    ~RobotLog();
  
  protected:
  
  private:
    bool createSystemLog();
    void chatterCallback(const rosgraph_msgs::Log::ConstPtr& msg);
    std::string tab_level(int x) const;
  private:
    DbManager dbManager;
    QSqlQuery query;
    std::string databasename;
    ros::NodeHandle n;
    ros::Subscriber subLog;
};

RobotLog::RobotLog()
{
  databasename = "/home/cfzhang/catkin_ws/my.db";
  if(!dbManager.createDataFile(QString::fromStdString(databasename)))
  {
    ROS_FATAL_STREAM("can not create database.");
    ros::shutdown();
  }
  if(!dbManager.openDataBase(QString::fromStdString(databasename)))
  {
    ROS_FATAL_STREAM("can not open database.");
    ros::shutdown();
  }
  query = QSqlQuery(dbManager.getDataBase());
  if(!dbManager.isExistTable("systemlog"))
  {
    if(!createSystemLog())
    {
      ROS_FATAL_STREAM("can not create systemlog table in database.");
      ros::shutdown();
    }
  }
  
  subLog = n.subscribe("/rosout_agg", 1000, &RobotLog::chatterCallback, this);
}

RobotLog::~RobotLog()
{
  dbManager.closeDataBase(); 
}
bool RobotLog::createSystemLog()
{
  
  bool ok = query.exec("CREATE TABLE IF NOT EXISTS systemlog (seq int,"
                                            "time varchar(225),"
                                            "frame_id varchar(225),"
                                            "level varchar(225),"
                                            "name varchar(225),"
                                            "msg varchar(225),"
                                            "file varchar(225),"
                                            "function varchar(225),"
                                            "line int,"
                                            "topics varchar(225))");

  return ok;
}


void RobotLog::chatterCallback(const rosgraph_msgs::Log::ConstPtr& msg)
{
  QSqlQuery query = QSqlQuery(dbManager.getDataBase());
  query.prepare("INSERT INTO systemlog (seq, time, frame_id, level, name, msg, file, function, line, topics) "
                           "VALUES (:seq, :time, :frame_id, :level, :name, :msg, :file, :function, :line, :topics)");
  
  //接收时间转换为C标准时间，再转换成string类型
  long pt = msg->header.stamp.sec;
  std::string time_str (ctime(&pt));

  //接收topics（string[]类型）并依次序写入topics_str（string类型）
  std::string topics_str;
  std::vector<std::string>::const_iterator it = msg->topics.begin();
  std::vector<std::string>::const_iterator end = msg->topics.end();
  for ( ; it != end; ++it )
  {
    const std::string& topic = *it;
    if ( it != msg->topics.begin() )
    {
      topics_str.append(",");
    }
    topics_str.append(topic);
  }

  query.bindValue(":seq", msg->header.seq);
  query.bindValue(":time", time_str.c_str());
  query.bindValue(":frame_id", msg->header.frame_id.c_str());
  query.bindValue(":level",tab_level(msg->level).c_str());
  query.bindValue(":name", msg->name.c_str());
  query.bindValue(":msg", msg->msg.c_str());
  query.bindValue(":file", msg->file.c_str());
  query.bindValue(":function", msg->function.c_str());
  query.bindValue(":line", msg->line);
  query.bindValue(":topics", topics_str.c_str());
  bool ok = query.exec();
  
}

std::string RobotLog::tab_level(int x) const
{
  using std::string;
  string s;
  switch (x)
  {
  case rosgraph_msgs::Log::FATAL:
    s = "FATAL";
    break;
  case rosgraph_msgs::Log::ERROR:
    s = "ERROR";
    break;
  case rosgraph_msgs::Log::WARN:
    s = "WARN";
    break;
  case rosgraph_msgs::Log::DEBUG:
    s = "DEBUG";
    break;
  case rosgraph_msgs::Log::INFO:
    s = "INFO";
    break;
  default:
    s = " ";
  }
  return s;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"robotslog");
  
  RobotLog r;
  // while(ros::ok());
  ros::spin();
  return 0;
}

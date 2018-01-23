#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QTimer>
#include <QHBoxLayout>
#include <QPushButton>
#include <QApplication>

#include <ros/ros.h>
#include <button_control/robotCtlMsg.h>

#include <boost/thread.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/SetMap.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private:
    QHBoxLayout *horizontalLayout;
    QPushButton *followBtn;
    QPushButton *backBtn;

    ros::NodeHandle nh_;
    ros::ServiceClient client;
    ros::Publisher _pub;
    QTimer* update_timer_;
    button_control::robotCtlMsg msg;
    ros::ServiceClient poseClient_;

    boost::shared_ptr<boost::thread> acwait_thread_;
    MoveBaseClient* ac_;
    move_base_msgs::MoveBaseGoal pack_desk_;    // 打包位置
    // move_base_msgs::MoveBaseGoal charge_port_;   // 充电位置
    bool pose_initialized_;

private Q_SLOTS:
    void onUpdate();
    void followBtnClicked();
    void backBtnClicked();
    bool sendGoal(const move_base_msgs::MoveBaseGoal& curr_goal);
    void acMonitor();
};

#endif // WIDGET_H

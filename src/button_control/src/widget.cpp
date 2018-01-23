#include "widget.h"
#include <button_control/robotCtl.h>

Widget::Widget(QWidget *parent) :
    QWidget(parent)
{
    QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
    sizePolicy1.setHorizontalStretch(0);
    sizePolicy1.setVerticalStretch(0);

    followBtn = new QPushButton(this);
    followBtn->setObjectName(QStringLiteral("followBtn"));
    sizePolicy1.setHeightForWidth(followBtn->sizePolicy().hasHeightForWidth());
    followBtn->setSizePolicy(sizePolicy1);
    QFont font1;
    font1.setPointSize(60);
    followBtn->setFont(font1);
    followBtn->setText(QApplication::translate("Widget", "FOLLOW", 0));

    backBtn = new QPushButton(this);
    backBtn->setObjectName(QStringLiteral("backBtn"));
    sizePolicy1.setHeightForWidth(backBtn->sizePolicy().hasHeightForWidth());
    backBtn->setSizePolicy(sizePolicy1);
    backBtn->setFont(font1);
    backBtn->setText(QApplication::translate("Widget", "BACK", 0));

    horizontalLayout = new QHBoxLayout(this);
    horizontalLayout->setSpacing(6);
    horizontalLayout->setContentsMargins(11, 11, 11, 11);
    horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
    horizontalLayout->addWidget(followBtn);
    horizontalLayout->addWidget(backBtn);

    update_timer_ = new QTimer(this);
    update_timer_->setInterval(20);
    update_timer_->start();

    connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));
    connect( followBtn, SIGNAL( pressed () ), this, SLOT( followBtnClicked() ));
    connect( backBtn,   SIGNAL( pressed () ), this, SLOT( backBtnClicked() ));

    pack_desk_.target_pose.header.frame_id = "map";
    pack_desk_.target_pose.header.stamp = ros::Time::now();
    pack_desk_.target_pose.pose.orientation.w = 1.0;

    pose_initialized_ = false;
    ac_ = new MoveBaseClient("move_base", true);
    // client = nh_.serviceClient<button_control::robotCtl>("robotControl");
    _pub = nh_.advertise<button_control::robotCtlMsg>("robotControl", 10);
    poseClient_ = nh_.serviceClient<nav_msgs::SetMap>("pose_initialized", 5);
}


// 发送目标给move_base服务器
bool Widget::sendGoal(const move_base_msgs::MoveBaseGoal& curr_goal)
{   
    nav_msgs::SetMap srv;
    while (!poseClient_.call(srv)){
        ros::Duration(1.0).sleep();
        ROS_WARN("Waiting for amcl server to come up ...");
    }

    pose_initialized_ = srv.response.success;
    if (!pose_initialized_){
        ROS_ERROR("Initial pose has not been set!");
        // ros::Duration(5.0).sleep();
        return false;
    }

    // cancel all goals before current time
    ac_->cancelGoalsAtAndBeforeTime(ros::Time(0));
    
    ROS_INFO("Heading for goal with world coords x: %.2f y:%.2f", curr_goal.target_pose.pose.position.x, curr_goal.target_pose.pose.position.y);

    ac_->sendGoal(curr_goal);
    return true;

}

//跟随按钮的回调函数
void Widget::followBtnClicked()
{
    // cancel all goals before current time
    try{
        ac_->cancelGoalsAtAndBeforeTime(ros::Time(0));
        ROS_INFO("Canceled all navi targets.");
    }
    catch(std::exception& ex){
        ROS_WARN("Failed canceling old target.");
    }

    //确认存在该robot的控制客户端信息，进行发送控制命令ID
    // button_control::robotCtl srv;
    // srv.request.cmdID = button_control::robotCtlRequest::FOLLOWEMODE;
    // if(client.call(srv))
    // {
    //   ROS_INFO_STREAM("follow cmd for  request success.");
    // }
    // else
    // {
    //   ROS_INFO_STREAM("follow cmd for  request fail.");
    // }

    msg.cmdID = button_control::robotCtlMsg::FOLLOWEMODE;
    _pub.publish(msg);

}
//返航按钮的回调函数
void Widget::backBtnClicked()
{

  // //确认存在该robot的控制客户端信息，进行发送控制命令ID
  // button_control::robotCtl srv;
  // srv.request.cmdID = button_control::robotCtlRequest::BACKMODE;
  // if(client.call(srv))
  // {
  //   ROS_INFO_STREAM("back cmd for  request success.");
  // }
  // else
  // {
  //   ROS_INFO_STREAM("back cmd for  request fail.");
  // }
    while(!ac_->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for move_base action server to come up");
    }

    msg.cmdID = button_control::robotCtlMsg::BACKMODE;
    _pub.publish(msg);

    if (sendGoal(pack_desk_)){
        acwait_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&Widget::acMonitor, this)));
    }

/*
    bool succeed = false;
    for (int i = 0; i < 10; ++i){
        if (sendGoal(pack_desk_)){
            succeed = true;
            break;
        }
    }
    if (!succeed){
        ROS_ERROR("FATAL ERROR! Failed after trying 10 times!");
    }
*/
}

void Widget::acMonitor(){
    ac_->waitForResult(); 

    if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Arrived!");
    }
    else{
        ROS_WARN("Failed with goal state: '%s', try another time.", ac_->getState().text_.c_str());
    }
}

void Widget::onUpdate()
{
    static unsigned int tick = 0;
    ros::spinOnce();
    // ROS_INFO_STREAM("hello world.");
    if (!ros::ok())
    {
        close();
    }
    // if(tick % 10 == 0)
    //     _pub.publish(msg);
    // tick ++;
}


Widget::~Widget()
{
    if (acwait_thread_)
        acwait_thread_->join();
    delete ac_;
}

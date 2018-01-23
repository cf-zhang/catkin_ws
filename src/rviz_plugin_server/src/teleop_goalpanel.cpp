#include <stdio.h>
#include <fstream>
#include <sstream>  
//#include <ros/console.h>


#include <QPushButton>
#include <QHeaderView>
#include <QFileDialog>
#include <QString>
#include <QMenu>
#include <QAction>
#include <QMessageBox>
#include <QInputDialog>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "teleop_goalpanel.h"
#include "map_server/GetMaps.h"
#include <lift_msgs/GetMapID.h>
#include <tf/transform_listener.h>

namespace rviz_plugin_server
{
#define TIMEOUT 60000


TeleopGoalPanel* TeleopGoalPanel::getGoalPanel()
{
  static TeleopGoalPanel *goalPanel = NULL;
  if(goalPanel == NULL)
  {
    goalPanel = new TeleopGoalPanel(0);
  }
  return goalPanel;
}




TeleopGoalPanel::TeleopGoalPanel( QWidget* parent )
  : rviz::Panel( parent ), TimeForRobotInfoRefresh(1000)
{
  // ROS_INFO_STREAM("TeleopGoalPanel is called. ");
  hbox_layout = new QHBoxLayout;
  robotBox = new QComboBox();
  // robotBox->addItem("AllRobot");
  hbox_layout->addWidget(robotBox);
//添加四个按钮，然后水平排列  
  hbt_layout = new QHBoxLayout;
  applybt = new QPushButton("Apply", this);
  addbt   = new QPushButton("Add", this);
  delbt   = new QPushButton("Del", this);
  savebt  = new QPushButton("Save", this);
//添加对应两种方式：从文件添加和从地图点击添加
//以下拉按钮的方式进行选择添加方式
  QMenu *add_menu = new QMenu(this); 
  QAction *addFileAction  = new QAction(add_menu);  
  QAction *addClickAction = new QAction(add_menu);
  addFileAction->setText("add target from file");  
  addClickAction->setText("add target from click");
  add_menu->addAction(addFileAction);  
  add_menu->addAction(addClickAction);  
  addbt->setMenu(add_menu);//设置菜单到添加按钮上
  //水平布局四个按钮
  hbt_layout->addWidget(applybt);  
  hbt_layout->addWidget(addbt);  
  hbt_layout->addWidget(delbt);
  hbt_layout->addWidget(savebt);
  
  //添加tableview，并进行属性设置
  h_layout = new QHBoxLayout;
  tv = new QTableView;
  model = new QStandardItemModel();
  //设置列数，列名字
  model->setColumnCount(2);
  model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("ID"));
  model->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("name"));
  model->setHeaderData(2,Qt::Horizontal,QString::fromLocal8Bit("type"));
  
  tv->setModel(model);
  //整行选中，单行选中，不可编辑
  tv->setSelectionBehavior(QAbstractItemView::SelectRows);
  tv->setEditTriggers(QAbstractItemView::DoubleClicked);//NoEditTriggers
  tv->setSelectionMode(QAbstractItemView::SingleSelection);
  tv->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
  //自动铺满，隐藏首列
  tv->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  tv->verticalHeader()->hide();
  h_layout->addWidget( tv );  

  //布局管理，将上面的控件进行垂直布局
  v_layout = new QVBoxLayout;
  v_layout->addLayout( hbox_layout );
  v_layout->addLayout( hbt_layout );
  v_layout->addLayout( h_layout );
  setLayout( v_layout );
  //用于控制点击选取目标点的标识，当选取了该功能之后置为true，选取点之后置false
  isWaitForClick = false;
  std::string defaultQR;
  //读取默认marker文件,并显示到表格中
  nh_.param<std::string>("defalutGoals", defaultQR, std::string("/home/cfzhang/catkin_ws/src/lift_racoon/marker/MarkerSave.txt"));
  // ROS_ERROR("%s",defaultQR.c_str());
  readQRTable(defaultQR, true);
  refreshQRTable();
  

  //设置信号和槽
  connect( applybt,        SIGNAL( pressed () ),  this, SLOT( applyBtnClicked() ));
  connect( addFileAction,  SIGNAL( triggered() ), this, SLOT( addFileBtnClicked() ));  
  connect( addClickAction, SIGNAL( triggered() ), this, SLOT( addClickBtnClicked() )); 
  connect( delbt,          SIGNAL( pressed () ),  this, SLOT( delBtnClicked() ));
  connect( savebt,         SIGNAL( pressed () ),  this, SLOT( saveBtnClicked() ));
  connect(tv->model(),     SIGNAL(itemChanged(QStandardItem*)),this,SLOT(editConfirmFun(QStandardItem*)));
  curMap=0;
  clientGetMapId_ = nh_.serviceClient<map_server::GetMapId>("getMapId");
  map_server::GetMapId srv;
  if(clientGetMapId_.call(srv))
  {
    curMap = srv.response.id;
  }
  // ROS_ERROR("%d",curMap);
  nav_click_sub = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 200, boost::bind(&TeleopGoalPanel::goalClickCallback, this, _1));
  mapId_sub_ = nh_.subscribe<std_msgs::Int64>("mapIdSub", 2, boost::bind(&TeleopGoalPanel::mapIdSubCallback, this, _1));

}


//描述：定时器事件函数，负责清理下线机器人的超时逻辑，机器人下拉框中条目的更新操作
//参数：@t： 自动传参，可以根据t->timerID()来区分具体是哪一个定时器触发了事件
//    @res：反馈内容
//返回值：无
void TeleopGoalPanel::robotInfoRefresh(std::vector<std::string> &robotsRemoved)//定时器时间
{// ROS_INFO_STREAM("hello ");
  //检查超时情况，如果超时结束，则清理robotinfo，反之则减少2
  //刷新robot下拉框中的显示
  // tgp->robotBox->clear();
  int index = -1;
  int time;
  std::map<std::string, RobotInfoValue>::iterator it;
  for(it=robotInfos.begin(); it!=robotInfos.end(); it++)
  {
    if(it->second.getTimeOut() <= 0)
    {
      index = robotBox->findText(QString::fromStdString(it->first));
      if(-1 == index)
      {
        ROS_INFO_STREAM("there is no item in combox");
        return;
      }
      robotBox->removeItem(index);
      robotInfos.erase(it);
      robotsRemoved.push_back(it->first);
    }
    else
    {
      time = it->second.getTimeOut() - (TimeForRobotInfoRefresh<<1);
      it->second.setTimeOut(time);
    }
  }
}




//描述：作为服务/robot_info回调函数中的goal处理入口，处理rviz界面中可选择的下拉框内容，超时时间设置，
//    可以修改TIMEOUT宏来调整每个机器人下线确认时间间隔，所有的robot信息都保存在了robotInfos中
//参数：
//    @req：请求内容
//    @res：反馈内容
//返回值：true：成功; false:失败
bool TeleopGoalPanel::robotInfoRegister(std::string robotName)
{
  std::map<std::string, RobotInfoValue>::iterator it;
  it = robotInfos.find(robotName);
  
  if(it == robotInfos.end())
  {//第一次请求，插入map。
    ros::Publisher pose_pub   = nh_.advertise<std_msgs::String>((robotName+"/set_goal"), 20);
    ros::Publisher mbgoal_pub = nh_.advertise<geometry_msgs::PoseStamped>((robotName+"/move_base_simple/goal"), 2);
    
    robotInfos.insert(std::map<std::string, RobotInfoValue>::value_type(robotName, RobotInfoValue(pose_pub, mbgoal_pub, TIMEOUT)));

    robotBox->addItem(QString::fromStdString(robotName));
  }
  else
  {//刷新时间
    it->second.setTimeOut(TIMEOUT);
  }

  return true;
}

//描述：作为话题mapIdSub的回调函数，接受到话题中消息后执行，将当前地图id获取到，
//  并根据该id进行刷新tv中的显示条目
//参数：@msg：接收到的消息
void TeleopGoalPanel::mapIdSubCallback(const std_msgs::Int64::ConstPtr& msg)
{
  // ROS_ERROR("%d",curMap);
  curMap = msg->data;
  // ROS_ERROR("%d",curMap);
  refreshQRTable();
}
//target要求可编辑进行修改选中点的名字
//需要记录编辑前的名字，以及获取修改后的名字
//采用删除旧节点重新插入新节点的方法进行操作
void TeleopGoalPanel::editConfirmFun(QStandardItem* item)
{
  int row= tv->currentIndex().row();
  if(-1 == row)
  {
    // ROS_INFO("please select a record for .");  
    return ;
  }
  QString curStr=item->data(Qt::DisplayRole).toString();

  //删除当前节点内存中的这个记录  
  QRMsgs::iterator it;
  it = marker_pose_.find(_beforeEdit[row]);
  // std::cout<<_beforeEdit[row]+" "<<it->second.getIsMarker()<<std::endl;
  if(it != marker_pose_.end() && !it->second.getIsMarker())
  {
    //先添加当前记录,然后删除以前的
    marker_pose_.insert(std::map<std::string, QRMsgsValue>::value_type(curStr.toStdString(), it->second));     
    // std::cout<<curStr.toStdString()<<std::endl;
    marker_pose_.erase(it);
  }
  refreshQRTable();
}



//描述：通过点击map中的点，来设置目标点的属性的回调函数，
//  通过确认窗口来获取坐标点名字，然后插入到内存中备用，最后刷新显示
//参数：@msg：在topic接收到的消息
void TeleopGoalPanel::goalClickCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{//for shortest road commented
  if(isWaitForClick)
  {
    // ROS_INFO("add click button is pressed!");

    char str[256];
    int len = 0;
    len += sprintf(str+len, "position:\n");
    len += sprintf(str+len, "\tx: %f\n", msg->pose.position.x);
    len += sprintf(str+len, "\ty: %f\n", msg->pose.position.y);
    len += sprintf(str+len, "\tz: %f\n", msg->pose.position.z);

    len += sprintf(str+len, "orientation:\n");
    len += sprintf(str+len, "\tx: %f\n", msg->pose.orientation.x);
    len += sprintf(str+len, "\ty: %f\n", msg->pose.orientation.y);
    len += sprintf(str+len, "\tz: %f\n", msg->pose.orientation.z);
    len += sprintf(str+len, "\tw: %f\n\n", msg->pose.orientation.w);
    len += sprintf(str+len, "Please input your goal name");
    bool isOK = false;
    QString text = QInputDialog::getText(NULL, "Save Goal",
                                                str,
                                                QLineEdit::Normal,
                                                "",&isOK); 
    if ( isOK && !text.isEmpty() )
    {
      int id__ = content.size();

      std::string name = boost::lexical_cast<std::string>(id__);
      QRMsgsValue value(*msg, false, curMap);
      //将这个goal添加到pose的map中
      // marker_pose_.insert(std::map<std::string, QRMsgsValue>::value_type(text.toStdString(), value));      
      marker_pose_.insert(std::map<std::string, QRMsgsValue>::value_type(name, value));   
      rviz_plugin_server::OptGoal srv;
      srv.request.pose  = *msg; 
      //还原成待保存的数据，添加到原始数据map中
      // content.insert(convertQuat2Matrix(text.toStdString(), srv.request.pose));
      content.insert(convertQuat2Matrix(name, srv.request.pose));
      //在rviz的插件中进行显示出来 inx
      refreshQRTable();

    }
    else
    {
      ROS_ERROR("unsaved clicked goal.");
    }
    isWaitForClick = false;
    refreshQRTable();
  }
}

//当应用按钮被按下，触发执行
//在这里获取当前选中的表格数据行，发送给指定的机器人
void TeleopGoalPanel::applyBtnClicked()
{
  ROS_INFO("apply button is pressed!");
  QString curRobot = robotBox->currentText();
  int row= tv->currentIndex().row();
  if(-1 == row)
  {
    ROS_INFO("please select a record for operation.");  
    return ;
  }
  QString str=model->data(model->index(row,1)).toString();
  QString type=model->data(model->index(row,2)).toString();
  if(type.toStdString() == "M")
  {//marker点的应用
    //将所选定的这条记录发送到map_server中，进行地图切换
    std_msgs::String msg;
    msg.data = str.toStdString();

    std::map<std::string, RobotInfoValue>::iterator it;
    it = robotInfos.find(curRobot.toStdString());
    if(it != robotInfos.end())
    {
      it->second.getPosePub().publish(msg);
    }
  }
  else if(type.toStdString() == "T")
  {//target点的应用
    //先从pose_map_中找到目标点信息，然后发布出去
    QRMsgs::iterator it;
    it = marker_pose_.find(str.toStdString());
    if(it != marker_pose_.end())
    {
      std::map<std::string, RobotInfoValue>::iterator ip;
      ip = robotInfos.find(curRobot.toStdString());
      if(ip != robotInfos.end())
      {
        ip->second.getMBGoalPub().publish(it->second.getPose());
      }      
    }
   
  }
}
//添加文件按钮被按下，触发执行
//在这里打开一个文件选择框，选择一个含有目标点信息文件
//当选中文件以后，点击确定，进行信息的提取，同步，显示操作
//支持一次性选择多个输入文件
void TeleopGoalPanel::addFileBtnClicked()
{
  ROS_INFO("add file button is pressed!");
   //打开一个文件选择框，获取文件信息
  QStringList fileNames = QFileDialog::getOpenFileNames(this,
                                tr("Open marker file"), ".",
                                tr("marker files (*.txt)"));
  if(!fileNames.isEmpty())
  {
    //将文件信息提取出来，
    for(int i = 0; i < fileNames.size(); i++)
    {
       readQRTable(fileNames[i].toStdString(), false);
       ROS_ERROR("open file for record");
    }
    //在rviz中表格内容的添加
    refreshQRTable();
  }
  else
  {
    ROS_INFO("the file cannot be choosen.");
  }
}

void TeleopGoalPanel::refreshQRTable()
{
  inx = 0;
  std::vector<std::string>().swap(_beforeEdit);
  model->removeRows(0,model->rowCount());
  QRMsgs::iterator it;
  for(it = marker_pose_.begin(); it != marker_pose_.end(); it++)
  {
    if(it->second.getMapId() == curMap || it->second.getIsMarker())
    {
      //在rviz中表格中添加着一个文件所对应信息的一条记录
      char id[20];
      //将获取到的地图信息显示到rviz表格中
      sprintf(id, "%d", inx);
      // ROS_INFO("%s",it->first.c_str());
    
      model->setItem(inx,0,new QStandardItem(id));
      //设置字符位置
      model->item(inx,0)->setTextAlignment(Qt::AlignCenter);
      model->setItem(inx,1,new QStandardItem(it->first.c_str()));
      model->item(inx,1)->setTextAlignment(Qt::AlignCenter);
      if(it->second.getIsMarker())
        model->setItem(inx,2,new QStandardItem(QString::fromLocal8Bit("M")));
      else
        model->setItem(inx,2,new QStandardItem(QString::fromLocal8Bit("T")));
      model->item(inx,2)->setTextAlignment(Qt::AlignCenter);
      _beforeEdit.push_back(it->first);
      inx++;
    }
  }
}
//当添加按钮的被按下，触发执行
//在这里打开一个文件选择框，进行地图文件的选择
//当选中文件以后，点击确定，在tableview中添加一行记录
void TeleopGoalPanel::addClickBtnClicked()
{
  // ROS_INFO("add click button is pressed!");
  QMessageBox msgBox;
  msgBox.setText("please click the map in 2D-Nav-Goal for a goal point.");
  // qsave = msgBox.addButton(QMessageBox::Save);
  QPushButton *ok = msgBox.addButton(QMessageBox::Ok);

  isWaitForClick = true;
  int ret = msgBox.exec();
}

//del按钮被按下，触发执行
//在这里获取当前选中的表格数据行，然后将这一条记录删除掉
//需要将lift_patrol中的记录删除
void TeleopGoalPanel::delBtnClicked()
{
  // ROS_INFO("delete button is pressed!");

  // ROS_INFO("del button is pressed!");
  int row= tv->currentIndex().row();
  if(-1 == row)
  {
    ROS_INFO("please select a record for operation.");  
    return ;
  }
  QString str=model->data(model->index(row,1)).toString();

  //删除lift_patrol中的地图信息
  rviz_plugin_server::OptGoal srv;
  srv.request.opt = 1;//1代表删除
  srv.request.label = str.toStdString();
   
  //删除当前节点内存中的这个记录  
  QRMsgs::iterator it;
  it = marker_pose_.find(srv.request.label);
  if(it != marker_pose_.end() && !it->second.getIsMarker())
  {//
    // //删除原始数据中的记录
    OrignContent::iterator oit;
    oit = content.find(srv.request.label);
    if(oit != content.end())
    {//找到，直接删除记录
      content.erase(oit);
    }

    marker_pose_.erase(it);
  }
  refreshQRTable();
}

//当save按钮被按下，触发执行
//保存当前系统中已经存在的所有目标点信息
void TeleopGoalPanel::saveBtnClicked()
{
  ROS_INFO("save button is pressed!");
  QString filename = QFileDialog::getSaveFileName(this,
                                  tr("Open Goal"), "",
                                  tr("Goal Files (*.txt)"));
  std::string file;
  if (!filename.isNull())
  {
    // ROS_ERROR("%s",filename.toStdString().c_str());
    file = filename.toStdString();
    std::ofstream outf; 
    outf.open(file.c_str());

     //删除原始数据中的记录
    OrignContent::iterator it;
    for(it = content.begin(); it != content.end(); it++)
    {
      outf<<"#"<<it->first<<std::endl;
            // outf<<"#"<<it->first<<it->second[16] <<std::endl;
      outf<< it->second[0]<<"\t" << it->second[1]<<"\t" << it->second[2]<<"\t" << it->second[3]<<std::endl;
      outf<< it->second[4]<<"\t" << it->second[5]<<"\t" << it->second[6]<<"\t" << it->second[7]<<std::endl;
      outf<< it->second[8]<<"\t" << it->second[9]<<"\t" << it->second[10]<<"\t" << it->second[11]<<std::endl;
      outf<< it->second[12]<<"\t" << it->second[13]<<"\t" << it->second[14]<<"\t" << it->second[15]<<std::endl;
    }
    outf.close();   
  }
  else
  {
      //点的是取消
  }
}

int TeleopGoalPanel::readQRTable(std::string filename, bool isMarker)
{
  int retVal = 0;
  int mapId = 0;
  // clear marker_pose_
  // QRMsgs().swap(marker_pose_);

	// open the file  
	std::ifstream inFile( filename.c_str(), std::ios_base::in);
	if (!inFile.is_open())
	{
		ROS_ERROR("file read failed");
		// ROS_ERROR("%s",filename.c_str());
    retVal = -1;
		return retVal;
	}
    
    // load the data
	std::istream_iterator<std::string> begin(inFile);    //the begin iterator of the  string flow
	std::istream_iterator<std::string> end;          //the end iterator of the  string flow  
	std::vector<std::string> vec(begin, end);      //save the file data to std::vector 
	std::vector<double> inData;
  char label[10];
  geometry_msgs::PoseStamped pose;
  tf::Transform transform;
  std::vector<std::string>::iterator it = vec.begin();
	for (; it != vec.end(); it++)
  {
		std::istringstream record(*it);
		std::string str;
		record >> str;
    // ROS_ERROR("%s",str.c_str());

    if (str[0] == '#')
    {
        // example: #11XA (floor 11, normal QR A), #11YB (floor 11, lift QR B), #12XC, #00ZA (inner lift QR A)
        str.copy(label, 4, 1);
        pose.header.frame_id = str.substr(1, 2);    // save floor index
        mapId = std::atoi(str.substr(5).c_str());//map id

        // ROS_INFO("%d, %s", __LINE__, label);
    }

		while (str[0] != '#')
    {
			double ch;
			ch = std::atof(str.c_str());
			inData.push_back(ch);
            // ROS_INFO("%d, %f", __LINE__, ch);
			break;
		}
    // ROS_WARN("%d, %f", __LINE__, inData.end());
    if (inData.size() == COLS*ROWS)
    {
      if(!isMarker)
      {
        inData.push_back(mapId);//默认将mapid放在vector的最后一个元素中
        content.insert(std::map<std::string, std::vector<double> >::value_type(std::string(label), inData));
      }
      transform = tf::Transform(tf::Matrix3x3(inData[0], inData[1], inData[2], 
                                              inData[4], inData[5], inData[6],
                                              inData[8], inData[9], inData[10]), 
                                  tf::Vector3(inData[3], inData[7], inData[11]));
      //transform.setOrigin(tf::Vector3(inData[3], inData[7], inData[11]));
      //transform.setRotation(tf::Quaternion());
      //pose.pose.position = geometry_msgs::Point(inData[3], inData[7], inData[11]);
      pose.pose.position.x = inData[3];
      pose.pose.position.y = inData[7];
      pose.pose.position.z = inData[11];
      // ROS_WARN("%f, %f, %f", inData[3], inData[7], inData[11]);
      // std::cout << inData[0] << " " <<  inData[1] << " " <<  inData[2] << " " <<  
      //                                         inData[4] << " " <<  inData[5] << " " <<  inData[6] << " " <<  
      //                                         inData[8] << " " <<  inData[9] << " " <<  inData[10] << std::endl;

      tf::quaternionTFToMsg(transform.getRotation(), pose.pose.orientation);

      QRMsgsValue value(pose, isMarker, mapId);
      //将这个goal添加到pose的map中
      marker_pose_.insert(std::map<std::string, QRMsgsValue>::value_type(label, value));    
      
      std::vector<double>().swap(inData);
      retVal++;
    }
	}
  inFile.close();

  return (retVal);
}
//将四元数转换成为矩阵，
std::map<std::string, std::vector<double> >::value_type 
TeleopGoalPanel::convertQuat2Matrix(std::string text, geometry_msgs::PoseStamped& msg)
{
  std::vector<double> data;
  //position
  double px = msg.pose.position.x;
  double py = msg.pose.position.y;
  double pz = msg.pose.position.z;
  //orientation
  double x = msg.pose.orientation.x;
  double y = msg.pose.orientation.y;
  double z = msg.pose.orientation.z;
  double w = msg.pose.orientation.w;
  double x2 = x * x, y2 = y * y, z2 = z * z;
  double xy = x * y, xz = x * z, yz = y * z;
  double wx = w * x, wy = w * y, wz = w * z;
  //一维
  data.push_back(1.0 - 2.0 * (y2+z2));  
  data.push_back(2.0 * (xy - wz));
  data.push_back(2.0 * (xz + wy));      
  data.push_back(px);
  //二维
  data.push_back(2.0 * (xy + wz));
  data.push_back(1.0 - 2.0 * (x2 + z2));
  data.push_back(2.0 * (yz - wx));
  data.push_back(py);
  //三维
  data.push_back(2.0*(xz-wy));
  data.push_back(2.0*(yz+wx));
  data.push_back(1.0-2.0 * (x2 + y2));
  data.push_back(pz);
  //四维
  data.push_back(0.0);
  data.push_back(0.0);
  data.push_back(0.0);
  data.push_back(1.0);
  //mapId
  data.push_back(curMap);
  std::map<std::string, std::vector<double> >::value_type ret(text, data);
  return ret;
}



TeleopGoalPanel::~TeleopGoalPanel()
{
  // ROS_INFO_STREAM("~TeleopGoalPanel is called. ");
  delete tv;
  delete model;
  delete applybt;
  delete addbt;
  delete delbt;
  delete savebt;

  delete hbox_layout;
  delete hbt_layout;
  delete h_layout;
  delete v_layout;

}
  


// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopGoalPanel::save( rviz::Config config ) const
{

}

// Load all configuration data for this panel from the given Config object.
void TeleopGoalPanel::load( const rviz::Config& config )
{

}

} // end namespace rviz_plugin_tutorials

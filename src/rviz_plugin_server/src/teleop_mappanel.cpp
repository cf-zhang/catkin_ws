/*
######## marker的应用是作用到lift上的。
######## 三列展示
######## lift和goal服务分别读marker和target文件。
######## 记录的增删查改（保存）只针对 goal记录生效
######## goal和marker分开，
######## goal的应用是作用到move_base。
######## goal要求可编。
######## map(只有lift会造成被动切换) 和 goal 要保证楼层和目标点一致。
*/
#include <stdio.h>
#include <ros/console.h>
#include <QPainter>

#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QHeaderView>
#include <QFileDialog>
#include <QString>
#include <geometry_msgs/Twist.h>
#include "teleop_mappanel.h"
#include "map_server/GetMaps.h"
#include <lift_msgs/GetMapID.h>


namespace rviz_plugin_server
{

TeleopMapPanel* TeleopMapPanel::getMapPanel()
{
  static TeleopMapPanel *mapPanel = NULL;
  if(mapPanel == NULL)
  {
    mapPanel = new TeleopMapPanel(0);
  }
  return mapPanel;
}



TeleopMapPanel::TeleopMapPanel( QWidget* parent )
  : rviz::Panel( parent ),TimeForMapRefresh(2000)
{
//添加三个按钮，然后水平排列
  hbt_layout = new QHBoxLayout;
  applybt = new QPushButton("Apply", this);
  addbt  = new QPushButton("Add", this);
  delbt  = new QPushButton("Del", this);

  hbt_layout->addWidget(applybt);  
  hbt_layout->addWidget(addbt);  
  hbt_layout->addWidget(delbt);  

  //添加tableview，并进行设置
  h_layout = new QHBoxLayout;  

  tv = new QTableView;
  model = new QStandardItemModel();
  //设置列数，列名字
  model->setColumnCount(2);
  model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("map ID"));
  model->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("map name"));
  
  tv->setModel(model);
  //整行选中，单行选中，不可编辑
  tv->setSelectionBehavior(QAbstractItemView::SelectRows);  
  tv->setEditTriggers(QAbstractItemView::NoEditTriggers); 
  tv->setSelectionMode(QAbstractItemView::SingleSelection);
  tv->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft); 
  //自动铺满，隐藏首列
  tv->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  tv->verticalHeader()->hide();
  h_layout->addWidget( tv );  

  //布局管理，将上面的控件进行垂直布局
  v_layout = new QVBoxLayout;
  v_layout->addLayout( hbt_layout );
  v_layout->addLayout( h_layout );
  setLayout( v_layout );


  //设置信号和槽
  connect( applybt, SIGNAL( pressed () ), this, SLOT( applyBtnClicked() ));
  connect( addbt,   SIGNAL( pressed () ), this, SLOT( addBtnClicked() ));
  connect( delbt,   SIGNAL( pressed () ), this, SLOT( delBtnClicked() ));
                    
  client_g = nh_.serviceClient<map_server::GetMaps>("get_maps");
  client_sw = nh_.serviceClient<lift_msgs::GetMapID>("static_map");
  //从地图服务器获取当前已经加载过的地图信息
  updateMap();
}
TeleopMapPanel::~TeleopMapPanel()
{

  delete tv;
  delete model;
  delete applybt;
  delete addbt;
  delete delbt;

  delete hbt_layout;
  delete h_layout;
  delete v_layout;
}
// 获取mapserver端的地图信息，并显示到rviz表格中
void TeleopMapPanel::updateMap()
{
  static bool isSucceed = true;
  map_server::GetMaps gmsrv;
  gmsrv.request.g = 1;
  //从mapserver获取地图信息
  if (client_g.call(gmsrv))
  {
    ROS_INFO("call service get_maps success");
    char id[20];
    for(inx = 0; inx < gmsrv.response.maps.size(); inx++)
    {//将获取到的地图信息显示到rviz表格中
      sprintf(id, "%d", inx);
      int pos = gmsrv.response.maps[inx].find_last_of('/');
      std::string mapname(gmsrv.response.maps[inx].substr(pos+1));
      model->setItem(inx,0,new QStandardItem(id));
      //     //设置字符颜色
      // model->item(i,0)->setForeground(QBrush(QColor(255, 0, 0)));
          //设置字符位置
      model->item(inx,0)->setTextAlignment(Qt::AlignCenter);
      model->setItem(inx,1,new QStandardItem(QString::fromLocal8Bit(mapname.c_str())));
    }
    // ROS_ERROR("%s", gmsrv.response.maps[i].c_str());
    isMapServerLaunched = true;
    isSucceed = true;
  }
  else
  {
    if(isSucceed)
    {
      isSucceed = false;
      ROS_ERROR("Failed to call service get_maps");
      model->clear();
    }
  }

}


//当应用按钮被按下，触发执行
//在这里获取当前选中的表格数据行，发送到mapserver中
void TeleopMapPanel::applyBtnClicked()
{
  // ROS_INFO("apply button is pressed!");
  int row= tv->currentIndex().row();
  if(-1 == row)
  {
    ROS_INFO("please select a record for operation.");  
    return ;
  }
  //将所选定的这条记录发送到map_server中，进行地图切换
  lift_msgs::GetMapID srv;
  srv.request.id = row;
  
  if (client_sw.call(srv))
  {
    ROS_INFO("success to call service for switch map.");
  }
  else
  {
    ROS_ERROR("Failed to call service get_maps_ID.");
  }
}
//当添加按钮被按下，触发执行
//在这里打开一个文件选择框，进行地图文件的选择
//当选中文件以后，点击确定，在tableview中添加一行记录
void TeleopMapPanel::addBtnClicked()
{

  // ROS_INFO("add button is pressed!");
  //打开一个文件选择框，获取文件信息
  QStringList fileNames = QFileDialog::getOpenFileNames(this,
                                tr("Open map yaml"), ".",
                                tr("yaml files (*.yaml)"));
  if(!fileNames.isEmpty())
  {
    map_server::GetMaps srv;
    //将文件信息提取出来，并且将信息同步到map_server中
    for(int i = 0; i < fileNames.size(); i++)
    {
      // ROS_INFO("%s", fileNames[i].toStdString().c_str());
      srv.request.g = 2;
      srv.request.mapname = fileNames[i].toStdString();
      //从mapserver获取地图信息
      if (! client_g.call(srv))
      {
        ROS_ERROR("Failed to add a map to map_server.");
        return;
      }

      //在rviz中表格中添加着一个文件所对应信息的一条记录
      char id[20];
      //将获取到的地图信息显示到rviz表格中
      sprintf(id, "%d", inx);
      int pos = fileNames[i].toStdString().find_last_of('/');
      std::string mapname(fileNames[i].toStdString().substr(pos+1));
      model->setItem(inx,0,new QStandardItem(id));
      //设置字符位置
      model->item(inx,0)->setTextAlignment(Qt::AlignCenter);
      model->setItem(inx,1,new QStandardItem(QString::fromLocal8Bit(mapname.c_str())));
      inx++;
    }
  }
  else
  {
    ROS_INFO("the file cannot be choosen.");
  }
}
//当删除按钮被按下，触发执行
//在这里获取当前选中的表格数据行，然后将这一条记录删除掉
//需要将mapserver中的记录删除
void TeleopMapPanel::delBtnClicked()
{
  // ROS_INFO("del button is pressed!");
  int row= tv->currentIndex().row();
  if(-1 == row)
  {
    ROS_INFO("please select a record for operation.");  
    return ;
  }
  //删除map_server中的地图信息
  map_server::GetMaps srv;
  srv.request.g = 3;
  srv.request.inx = row;
  //从mapserver获取地图信息
  if (! client_g.call(srv))
  {
    ROS_ERROR("Failed to delete map_server's map.");
    return;
  }

  //删除rviz中的记录
  model->removeRow(row);
  inx -= 1;
  //将表格中的map_id进行更新。
  char id[20];
  for(int i = 0; i < inx; i++)
  {//将获取到的地图信息显示到rviz表格中
    sprintf(id, "%d", i);
    model->setItem(i,0,new QStandardItem(id));
    model->item(i,0)->setTextAlignment(Qt::AlignCenter);
  } 
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void TeleopMapPanel::save( rviz::Config config ) const
{}
// Load all configuration data for this panel from the given Config object.
void TeleopMapPanel::load( const rviz::Config& config )
{}
} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
// #include <pluginlib/class_list_macros.h>
// PLUGINLIB_EXPORT_CLASS(rviz_plugin_server::TeleopMapPanel,rviz::Panel )


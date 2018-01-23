#include "batterybutton.h"
#include <QPixmap>
#include <QDebug>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
BatteryButton::BatteryButton(QWidget *parent) : QPushButton(parent)
{
    package_path_ = ros::package::getPath("robot_client_interface");
    QString imgName = QString::fromStdString( (fs::path(package_path_ + "/image/battery100.png").string()));
    setAutoFillBackground(true);
    setFixedSize(100,100);
    setIconSize(QSize(100, 100));
    setIcon(QIcon(imgName));
    setFlat(true);
    setStyleSheet("background-color: rgb(37, 109, 167);");

    QFont font("Microsoft YaHei", 14, QFont::Black);
    batteryMenu = new QMenu;
    batteryMenu->setFont(font);
    batteryMenu->setFixedWidth(parent->width()-10);

    batteryMenu->setStyleSheet(
                " QMenu {\
                background-color: white; /* sets background of the menu 设置整个菜单区域的背景色，我用的是白色：white*/\
                border: 1px solid white;/*整个菜单区域的边框粗细、样式、颜色*/\
                }\
                QMenu::item {\
                    /* sets background of menu item. set this to something non-transparent\
                        if you want menu color and menu item color to be different */\
                    background-color: transparent;\
                    padding:8px 32px;/*设置菜单项文字上下和左右的内边距，效果就是菜单中的条目左右上下有了间隔*/\
                    margin:0px 8px;/*设置菜单项的外边距*/\
                    border-bottom:1px solid #DBDBDB;/*为菜单项之间添加横线间隔*/\
                }\
                QMenu::item:selected { /* when user selects item using mouse or keyboard */\
                    background-color: #2dabf9;/*这一句是设置菜单项鼠标经过选中的样式*/\
                }");
    //add items to menu
    persent = new QAction("剩余电量：100%",this);
    remains = new QAction("剩余时长：4 小时",this);
    
    batteryMenu->addAction(persent);
    batteryMenu->addAction(remains);

    //    settingMenu->addSeparator();
    setFixedWidth(parent->width());
    setMenu(batteryMenu);


    //timer for periodically update the wifi signal strenth
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    timer->start(1000);
}



BatteryButton::~BatteryButton(){
    delete img;
//    delete
    delete timer;

}
//timer event for periodically get wifi signal strenth and
//change the image for the wifi lable. rate 1Hz
void BatteryButton::onTimeout(){

    QString imgName = QString::fromStdString( (fs::path(package_path_ + "/image/battery100.png").string()));
    static int power = 0;
    power %= 100;
    QString qrc = QString(imgName);

//    int status;
//    status = 0;//////////////////////
//    if(0 < status){
        qrc = getQrcImage(power);/////
//    }
//    qDebug()<<" "<<power<<"\n";
    //set backgroud image and scaled with the lable
    setIcon(QIcon(qrc));
    power++;
}

QString BatteryButton::getQrcImage(int power){
    if(power > 90)
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery100.png").string()));
    else if(power <= 90 && power > 80) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery90.png").string()));
    else if(power <= 80 && power > 70) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery80.png").string()));
    else if(power <= 70 && power > 60) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery70.png").string()));
    else if(power <= 60 && power > 50) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery60.png").string()));
    else if(power <= 50 && power > 40) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery50.png").string()));
    else if(power <= 40 && power > 30) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery40.png").string()));
    else if(power <= 30 && power > 20) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery30.png").string()));
    else if(power <= 20 && power > 10) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery20.png").string()));
    else if(power <= 10 && power > 5) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery10.png").string()));
    else if(power <= 5 && power >= 0) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/battery5.png").string()));
    else
        QString::fromStdString( (fs::path(package_path_ + "/image/batteryerror.png").string()));
}

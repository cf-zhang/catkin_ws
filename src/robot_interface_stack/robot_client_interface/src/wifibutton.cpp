#include "wifibutton.h"
#include <QDebug>
#include <QString>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
WifiButton::WifiButton(QString wifiname, QWidget *parent) : QPushButton(parent),wifiName(wifiname){
    setAutoFillBackground(true);
    setFixedSize(100,100);
    setIconSize(QSize(100, 100));

    package_path_ = ros::package::getPath("robot_client_interface");
    QString imgName = QString::fromStdString( (fs::path(package_path_ + "/image/wifi0.png").string()));

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
    persent = new QAction("关于本机",this);
    remains = new QAction("重启",this);

    batteryMenu->addAction(persent);
    batteryMenu->addAction(remains);

    //    settingMenu->addSeparator();
    setFixedWidth(parent->width());
    setMenu(batteryMenu);



    wifi=wifi_scan_init(wifiName.toStdString().c_str());

    //timer for periodically update the wifi signal strenth
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    timer->start(1000);
}


WifiButton::~WifiButton(){
    delete img;
    delete timer;
    //free the library resources
    wifi_scan_close(wifi);
}
//timer event for periodically get wifi signal strenth and
//change the image for the wifi lable. rate 1Hz
void WifiButton::onTimeout(){
    QString qrc = QString::fromStdString( (fs::path(package_path_ + "/image/wifi0.png").string()));

    int status;
    status = wifi_scan_station(wifi, &station);
    if(0 < status){
        qrc = getQrcImage(station.signal_dbm);
    }

    setIcon(QIcon(qrc));


}

QString WifiButton::getQrcImage(int strength){
    if(strength <= -30 && strength > -67) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/wifi4.png").string()));
    else if(strength <= -67 && strength > -70) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/wifi3.png").string()));
    else if(strength <= -70 && strength > -80)
        return QString::fromStdString( (fs::path(package_path_ + "/image/wifi2.png").string()));
    else if(strength <= -80 && strength > -90) 
        return QString::fromStdString( (fs::path(package_path_ + "/image/wifi1.png").string()));
    else 
        return QString::fromStdString( (fs::path(package_path_ + "/image/wifi0.png").string()));
}


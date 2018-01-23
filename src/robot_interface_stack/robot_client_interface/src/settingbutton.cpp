#include "settingbutton.h"

#include "power.h"
#include <QPalette>
#include <QFont>
#include <QDebug>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
SettingButton::SettingButton(QWidget *parent):QPushButton(parent){
    package_path_ = ros::package::getPath("robot_client_interface");
    setAutoFillBackground(true);
    setFixedSize(100,100);
    setIconSize(QSize(100, 100));
    QString imgName = QString::fromStdString( (fs::path(package_path_ + "/image/setting.png").string()));
    setIcon(QIcon(imgName));

    setStyleSheet("background-color: rgb(37, 109, 167);");


    QFont font("Microsoft YaHei", 14, QFont::Black);
    settingMenu = new QMenu;
    settingMenu->setFont(font);
    settingMenu->setFixedWidth(parent->width()-10);

    settingMenu->setStyleSheet(
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
    aboutDev = new QAction("关于本机",this);
    restartDev = new QAction("重启",this);
    shutdownDev = new QAction("关机",this);

    settingMenu->addAction(aboutDev);
    settingMenu->addAction(restartDev);
    settingMenu->addAction(shutdownDev);
//    settingMenu->addSeparator();
    setFixedWidth(parent->width());
    setMenu(settingMenu);

    connect(aboutDev, SIGNAL(triggered(bool)), this, SLOT(aboutDevice()));
    connect(shutdownDev, SIGNAL(triggered(bool)), this, SLOT(shutDownDevice()));
    connect(restartDev, SIGNAL(triggered(bool)), this, SLOT(restartDevice()));
}

SettingButton::~SettingButton(){
    delete aboutDev;
    delete restartDev;
    delete shutdownDev;
    delete settingMenu;
}

void SettingButton::aboutDevice(){
    AboutThis about;

    about.exec();
}


void SettingButton::shutDownDevice(){
//    qDebug()<<"in shutdown device\n";
    Power::automatic = true;
    Power::shutdown();
}


void SettingButton::restartDevice(){
//    qDebug()<<"in restart device\n";
    Power::automatic = true;
    Power::reboot();
}

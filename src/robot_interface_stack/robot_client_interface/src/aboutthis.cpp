#include "aboutthis.h"
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <QString>
#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QDesktopWidget>
namespace fs = boost::filesystem;
AboutThis::AboutThis(QWidget *parent) : QDialog(parent)
{

    setWindowTitle("关于本机");

    // QPushButton *oKBtn = addButton(QMessageBox::Ok);
    // setText("this is a message box");
    // std::string package_path_ = ros::package::getPath("robot_client_interface");
    // QString iconPath = QString::fromStdString( (fs::path(package_path_ + "/image/setting.png").string()));
    // // ROS_ERROR_STREAM(" "<<help_path_.toStdString());
    // setIconPixmap(QPixmap(iconPath));

//    if(customBox.clickedButton()==oKBtn)
//        label->setText("custom message box / oKBtn");
    QTextBrowser *tb = new 	QTextBrowser(this);
    tb->setFixedSize(480,680);
    QFile file("/home/cfzhang/start.sh");
    if(!file.open(QFile::ReadOnly | QFile::Text))
        qDebug()<<"can not open file";
    QTextStream in(&file);
    tb->setHtml(in.readAll());
}
AboutThis::~AboutThis(){

}

void AboutThis::showEvent(QShowEvent* event) {
// QMessageBox::showEvent(event);
    // setFixedSize(640, 480);
    // resize(900,500);
    move((QApplication::desktop()->width() - width())/2,
        (QApplication::desktop()->height() - height())/2);
}
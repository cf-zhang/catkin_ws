#include "batterylabel.h"
#include <QPixmap>
#include <QDebug>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
BatteryLabel::BatteryLabel(QWidget *parent) : QLabel(parent)
{
    package_path_ = ros::package::getPath("robot_client_interface");
    setFixedSize(100,100);
    //set the initial battery image
    img = new QImage();
    QString imgName = QString::fromStdString( (fs::path(package_path_ + "/image/battery100.png").string()));
    if(img->load(imgName))
    {
//        qDebug()<<img->width()<<" "<<img->height();
        setPixmap(QPixmap::fromImage(img->scaled(size())));
    }

    //timer for periodically update the wifi signal strenth
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    timer->start(1000);
}



BatteryLabel::~BatteryLabel(){
    delete img;
//    delete
    delete timer;

}
//timer event for periodically get wifi signal strenth and
//change the image for the wifi lable. rate 1Hz
void BatteryLabel::onTimeout(){
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
    if(img->load(qrc))
    {
        setPixmap(QPixmap::fromImage(img->scaled(size())));
    }
    power++;
}

QString BatteryLabel::getQrcImage(int power){

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

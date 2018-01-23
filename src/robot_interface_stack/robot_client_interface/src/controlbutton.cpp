#include "controlbutton.h"
#include "mainwidget.h"
#include <QDebug>
#include <QBitmap>
#include <QPainter>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
ControlButton::ControlButton(QWidget *parent) : QPushButton(parent)
{
    package_path_ = ros::package::getPath("robot_client_interface");
    setStyleSheet("QPushButton{border:5px;}");//这句务必加上，否则看到的就是矩形了，而不是不规则图形了

    setFlat(true);
    QString imgName = QString::fromStdString( (fs::path(package_path_ + "/image/arrow1.png").string()));
    QPixmap *pm = new QPixmap(imgName);
    setIcon(*pm);
    setIconSize(QPixmap(imgName).size());

    //set the circle view
    QBitmap bmp(QPixmap(imgName).size());
    bmp.fill();
    QPainter p(&bmp);
    p.setPen(Qt::NoPen);
    p.setBrush(Qt::black);
    p.drawEllipse(bmp.rect());
//    setMask(bmp);


    //timer for periodically update the wifi signal strenth
    curIcon = 1;
    isActive = false;
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
}

ControlButton::~ControlButton()
{
    delete timer;
}

void ControlButton::startRotate()
{
    timer->start(200);
}
void ControlButton::stopRotate()
{
    timer->stop();
}


void ControlButton::onTimeout(){
    QString imgName = QString::fromStdString( (fs::path(package_path_ + "/image/arrow%1.png").string()));
//        qDebug()<<curIcon;
    QString qrc= imgName.arg(curIcon);
    setIcon(QPixmap(qrc));

    curIcon %= 3;
    curIcon++;

}

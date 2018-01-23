#include "viewlabel.h"
#include <QBitmap>
#include <QPainter>
#include <QDebug>
#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
ViewLabel::ViewLabel(QWidget *parent) : QLabel(parent)
{
    package_path_ = ros::package::getPath("robot_client_interface");
    setFixedSize(700,300);
    setWindowFlags(Qt::FramelessWindowHint);
    setAutoFillBackground(true);
//    setFixedWidth(140);
    QPalette palette;
//    palette.setColor(QPalette::Background,QColor(37, 109, 167));
    QString imgName = QString::fromStdString( (fs::path(package_path_ + "/image/standby.png").string()));
    palette.setBrush(QPalette::Background, QBrush(QPixmap(imgName)));
    setPalette(palette);
    QBitmap bmp(this->size());

    bmp.fill();

    QPainter p(&bmp);

    p.setPen(Qt::NoPen);

    p.setBrush(Qt::black);

    p.drawRoundedRect(bmp.rect(),20,20);

    setMask(bmp);
}


ViewLabel::~ViewLabel(){

}

void ViewLabel::setBackgroundPixmap(QString qrc){
    QPalette palette;
//    palette.setColor(QPalette::Background,QColor(37, 109, 167));
    palette.setBrush(QPalette::Background, QBrush(QPixmap(qrc)));
    setPalette(palette);
//    qDebug()<<"in set pixmap\n";
//    show();
}

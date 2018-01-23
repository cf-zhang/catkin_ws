#include "timelabel.h"
#include <QFont>
#include <QTime>
TimeLabel::TimeLabel(QWidget *parent):QLabel(parent){
//    resize(100,50);
    setFixedSize(100,40);
    //font set
    QFont font("Microsoft YaHei", 28, QFont::Bold);
    setFont(font);
    QPalette pa;//color set
    pa.setColor(QPalette::WindowText,Qt::white);
    setPalette(pa);
    //init the time when start
    setText(QTime::currentTime().toString("hh:mm"));

    //timer for periodically update the wifi signal strenth
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    timer->start(1000);//min /
}

TimeLabel::~TimeLabel(){
    delete timer;
}


void TimeLabel::onTimeout(){
//    setText("12:33:25 dddd");
    setText(QTime::currentTime().toString("hh:mm"));
}

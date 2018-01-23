#include "customsplash.h"


CustomSplash::CustomSplash(QWidget *parent)
    : QWidget(parent)
{
    this->setWindowFlags(Qt::FramelessWindowHint
                         | Qt::WindowSystemMenuHint
                         | Qt::WindowMinMaxButtonsHint
                         | Qt::WindowStaysOnTopHint);
    this->setAttribute(Qt::WA_TranslucentBackground);//背景透明
    //this->setAttribute(Qt::WA_DeleteOnClose);
    this->resize(500,300);

    //设置动画背景
    movie = new QMovie();
    movie->setFileName(":/gifs/start1.gif");
    movie->setScaledSize(QSize(500,300));
    QLabel *blkPic = new QLabel(this);
    blkPic->setMovie(movie);
    blkPic->setFixedSize(500,300);


    this->show();
}

CustomSplash::~CustomSplash()
{
    delete movie;
    delete blkPic;
}



void CustomSplash::startSplash()
{
    movie->start();
}

void CustomSplash::stopSplash()
{
    movie->stop();
    this->close();
}

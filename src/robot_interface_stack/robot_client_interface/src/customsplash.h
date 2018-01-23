#ifndef CUSTOMSPLASH_H
#define CUSTOMSPLASH_H

#include <QWidget>
#include <QMovie>
#include <QLabel>
class CustomSplash : public QWidget
{
    Q_OBJECT

public:
    CustomSplash(QWidget *parent = 0);
    ~CustomSplash();
    enum SplashTextOrientation{LeftTop,RightTop,LeftBottom,RightBottom};
public:
    void startSplash(); //开启启动动画
    void stopSplash();  //停止启动动画
private:
    QMovie *movie;
    QLabel *blkPic;
};

#endif // CUSTOMSPLASH_H

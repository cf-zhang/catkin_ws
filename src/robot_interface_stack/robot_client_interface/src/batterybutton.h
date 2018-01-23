#ifndef BATTERYBUTTON_H
#define BATTERYBUTTON_H

#include <QObject>
#include <QWidget>
#include <QImage>
#include <QPushButton>
#include <QTimer>
#include <QMenu>
#include <QAction>
class BatteryButton : public QPushButton
{
    Q_OBJECT
public:
    BatteryButton(QWidget *parent = nullptr);
    virtual ~BatteryButton();
private:
    QString getQrcImage(int power);


private Q_SLOTS:
    void onTimeout();
private:
    QImage *img;
    QTimer *timer;
    QMenu *batteryMenu;
    QAction *persent;
    QAction *remains;
    std::string package_path_;
};

#endif // BATTERYBUTTON_H


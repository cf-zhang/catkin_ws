#ifndef WIFIBUTTON_H
#define WIFIBUTTON_H
#include "wifi_scan.h"
#include <QTimer>
#include <QImage>
#include <QObject>
#include <QMenu>
#include <QAction>

#include <QPushButton>
class WifiButton : public QPushButton
{
    Q_OBJECT
public:
    WifiButton(QString wifiname="wlan0", QWidget *parent = nullptr);
    virtual ~WifiButton();
private:
    QString getQrcImage(int strength);

public Q_SLOTS:
    void onTimeout();
private:
    QMenu *batteryMenu;
    QAction *persent;
    QAction *remains;
    std::string package_path_;
    QString wifiName;
    QImage *img;
    QTimer *timer;
    struct wifi_scan *wifi;
    struct station_info station;
};

#endif // WIFIBUTTON_H

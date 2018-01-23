#ifndef TOPBARWIDGET_H
#define TOPBARWIDGET_H
#include <QVBoxLayout>
#include <QWidget>
#include "wifibutton.h"
#include "timelabel.h"
#include "batterybutton.h"
#include "settingbutton.h"
class TopBarWidget : public QWidget
{
    Q_OBJECT
public:
    explicit TopBarWidget(QWidget *parent = nullptr);
    virtual ~TopBarWidget();

private:
    QVBoxLayout *layout;
    WifiButton *wifibutton;
    TimeLabel *timelabel;
    BatteryButton *batterbutton;
    SettingButton *settingbutton;
};

#endif // TOPBARWIDGET_H

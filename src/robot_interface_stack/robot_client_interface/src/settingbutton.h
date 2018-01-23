#ifndef SETTINGBUTTON_H
#define SETTINGBUTTON_H
#include "aboutthis.h"
#include <QObject>
#include <QImage>
#include <QWidget>
#include <QPushButton>
#include <QMenu>
#include <QAction>
class SettingButton : public QPushButton
{
    Q_OBJECT
public:
    SettingButton(QWidget *parent = nullptr);
    virtual ~SettingButton();
private:
    std::string package_path_;
    QMenu *settingMenu;
    QAction *aboutDev;
    QAction *shutdownDev;
    QAction *restartDev;

public Q_SLOTS:
    void shutDownDevice();
    void restartDevice();
    void aboutDevice();
};

#endif // SETTINGBUTTON_H


#include "topbarwidget.h"

TopBarWidget::TopBarWidget(QWidget *parent) : QWidget(parent)
{
    setAutoFillBackground(true);
    setFixedWidth(140);
    QPalette palette;
    palette.setColor(QPalette::Background,QColor(37, 109, 167));
//    palette.setBrush(QPalette::Background, QBrush(QPixmap(":/image/timg.png")));
    setPalette(palette);
    //top bars
    wifibutton = new WifiButton("wlp2s0",this);
    timelabel = new TimeLabel(this);
    batterbutton = new BatteryButton(this);
    settingbutton = new SettingButton(this);
    //layout the bars with 30 spacing
    layout = new QVBoxLayout();
    layout->setSpacing(50);

    layout->addWidget(timelabel,0,Qt::AlignTop | Qt::AlignCenter);
    layout->addWidget(wifibutton,0,Qt::AlignTop | Qt::AlignCenter);
    layout->addWidget(batterbutton, 0, Qt::AlignTop | Qt::AlignCenter);
    layout->addStretch();
    layout->addWidget(settingbutton,0,Qt::AlignBottom | Qt::AlignCenter);

    setLayout(layout);
}

TopBarWidget::~TopBarWidget(){
    delete wifibutton;
    delete timelabel;
    delete settingbutton;
    delete batterbutton;
    delete layout;
}

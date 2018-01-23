#ifndef WIDGET_H
#define WIDGET_H
#include "mainwidget.h"
#include "topbarwidget.h"

#include <QWidget>
#include <QLabel>
#include <QSplitter>
class SplitterManager
{

public:
    SplitterManager();
    ~SplitterManager();
private:
    QSplitter *splitter;
    TopBarWidget *topWidget;
    MainWidget *mainWidget;
};

#endif // WIDGET_H

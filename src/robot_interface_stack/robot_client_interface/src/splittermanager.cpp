#include "splittermanager.h"


SplitterManager::SplitterManager()
{
    splitter = new QSplitter(Qt::Horizontal,0);
    QList<int> widgetSizes;
    widgetSizes << 120 << 520;
    splitter->setSizes(widgetSizes);
    splitter->setHandleWidth(1);


    topWidget = new TopBarWidget();
    mainWidget = new MainWidget();

    splitter->addWidget(topWidget);
    splitter->addWidget(mainWidget);

    splitter->setStyleSheet("QSplitter::handle { background-color: black }");
    QSplitterHandle *splitterHandle = splitter->handle(1);

    if(splitterHandle)
    {
        //Disable the Middle Line, it can't adjust.
        splitterHandle->setDisabled(true);
    }
    splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    splitter->setStretchFactor(0,1);
    splitter->setStretchFactor(1,24);
    splitter->showFullScreen();
}

SplitterManager::~SplitterManager()
{
    delete topWidget;
    delete mainWidget;
    delete splitter;
}

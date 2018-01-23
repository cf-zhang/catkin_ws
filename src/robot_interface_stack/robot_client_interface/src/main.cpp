#include "splittermanager.h"
#include "customsplash.h"
#include "mainwidget.h"
#include "topbarwidget.h"
#include "mainwidget.h"
#include <QApplication>
#include <QSplitter>
#include <QTime>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);



//    CustomSplash cs;
//    cs.startSplash();
//    QTime t;
//    t.start();
//    while(t.elapsed()<10000)
//        QCoreApplication::processEvents();
//    cs.stopSplash();
    SplitterManager sm;

    return a.exec();
}

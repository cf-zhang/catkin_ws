#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "qnode.hpp"
int main(int argc, char *argv[])
{
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));

//    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);
    QNode node(argc,argv);
    node.init();

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("/home/cfzhang/xcode/catkin_qt/src/qmldemo/resources/main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;

    return app.exec();
}

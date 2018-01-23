#ifndef VIEWLABEL_H
#define VIEWLABEL_H
#include <QString>
#include <QLabel>
class ViewLabel :public QLabel
{
    Q_OBJECT
public:
    ViewLabel(QWidget *parent = nullptr);
    virtual ~ViewLabel();
    void setBackgroundPixmap(QString qrc);
private:
    std::string package_path_;


};

#endif // VIEWLABEL_H

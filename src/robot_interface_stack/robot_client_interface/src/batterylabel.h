#ifndef BATTERYLABEL_H
#define BATTERYLABEL_H
#include <QImage>
#include <QLabel>
#include <QTimer>

class BatteryLabel : public QLabel
{
    Q_OBJECT
public:
    BatteryLabel(QWidget *parent = nullptr);
    virtual ~BatteryLabel();
private:
    QString getQrcImage(int power);

public Q_SLOTS:
    void onTimeout();
private:
    QImage *img;
    QTimer *timer;
    std::string package_path_;
};

#endif // BATTERYLABEL_H

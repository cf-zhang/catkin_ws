#ifndef TIMELABEL_H
#define TIMELABEL_H

#include <QObject>
#include <QLabel>
#include <QTimer>

class TimeLabel : public QLabel
{
    Q_OBJECT
public:
    TimeLabel(QWidget *parent = nullptr);
    virtual ~TimeLabel();
private:
    QTimer *timer;

public Q_SLOTS:
    void onTimeout();
};

#endif // TIMELABLE_H

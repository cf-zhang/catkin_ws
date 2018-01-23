#ifndef ControlButton_H
#define ControlButton_H

#include <QWidget>
#include <QPushButton>
#include <QObject>
#include <QTimer>
class ControlButton : public QPushButton
{
    Q_OBJECT
public:
    explicit ControlButton(QWidget *parent = nullptr);
    virtual ~ControlButton();
    void startRotate();
    void stopRotate();

public Q_SLOTS:
    void onTimeout();
private:
    QTimer *timer;
    int curIcon;
    bool isActive;
    std::string package_path_;
};

#endif // ControlButton_H


#ifndef ABOUTTHIS_H
#define ABOUTTHIS_H

#include <QTextBrowser>
#include <QDialog>
#include <QPushButton>
#include <QLabel>
#include <QShowEvent>
class AboutThis : public QDialog
{
    Q_OBJECT
public:
    explicit AboutThis(QWidget *parent = nullptr);
    virtual ~AboutThis();
    void showEvent(QShowEvent* event);
public Q_SLOTS:
//    void showAboutThis();
private:
    QTextBrowser *tb;
};

#endif // ABOUTTHIS_H

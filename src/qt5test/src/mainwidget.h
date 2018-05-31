#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include "../include/testgui/qnode.hpp"
#include "widget.h"
namespace Ui {
class MainWidget;
}

class MainWidget : public QWidget
{
  Q_OBJECT

public:
  explicit MainWidget(int argc,char *argv[],QWidget *parent = 0);
  ~MainWidget();

private Q_SLOTS:
  void on_pushButton_4_clicked();

private Q_SLOTS:
  void on_pushButton_7_clicked();

  void on_pushButton_clicked();

  void on_pushButton_2_clicked();

private:
  Ui::MainWidget *ui;
  testgui::QNode qnode;
  log_panel::Widget *w;

};

#endif // MAINWIDGET_H

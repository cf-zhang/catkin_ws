#include "mainwidget.h"
#include "ui_mainwidget.h"

MainWidget::MainWidget(int argc,char *argv[],QWidget *parent) :
  QWidget(parent),
  ui(new Ui::MainWidget),
  qnode(argc,argv)
{
  ui->setupUi(this);
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  qnode.init();
}

MainWidget::~MainWidget()
{
  delete ui;
}


void MainWidget::on_pushButton_4_clicked()
{
  int value;
  if(qnode.toAClientCall(&value))
  {
    ui->lineEdit_5->setText(QString::number(value));
  }
}

void MainWidget::on_pushButton_7_clicked()
{
  int value;
  if(qnode.toBClientCall(&value))
  {
    ui->lineEdit_6->setText(QString::number(value));
  }
}

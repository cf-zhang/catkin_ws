#include "mainwidget.h"
#include "ui_mainwidget.h"

MainWidget::MainWidget(int argc,char *argv[],QWidget *parent) :
  QWidget(parent),
  ui(new Ui::MainWidget),
  qnode(argc,argv)
{
  ui->setupUi(this);
  qnode.init();
  QGridLayout *grid = new QGridLayout(ui->page_4);

  w = new log_panel::Widget();
  grid->addWidget(w,0,0,1,1);
//  grid->addItem(ui->pushButton_2);
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

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

void MainWidget::on_pushButton_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->page_4);
}

void MainWidget::on_pushButton_2_clicked()
{
    ui->stackedWidget->setCurrentWidget(ui->page_3);
}

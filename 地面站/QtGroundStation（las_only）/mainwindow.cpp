#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->exitBtn,SIGNAL(clicked()),this,SLOT(close()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_ShowBtn_clicked()
{
    this->hide();
    window_selected = 1;
    ShowWindow *showwindow=new ShowWindow(this);
    showwindow->show();
}

void MainWindow::on_CollectionBtn_clicked()
{
    this->hide();
    window_selected = 2;
    CollectRecoverWindow *collectrecoverwindow=new CollectRecoverWindow(this);
    collectrecoverwindow->show();
}

void MainWindow::on_FilterBtn_clicked()
{
    this->hide();
    window_selected = 3;
    FilterWindow *filterwindow=new FilterWindow(this);
    filterwindow->show();
}



void MainWindow::on_pushButton_6_clicked()
{
    this->hide();
    window_selected = 6;
    registration *reg_window = new registration(this);
    reg_window->show();
}

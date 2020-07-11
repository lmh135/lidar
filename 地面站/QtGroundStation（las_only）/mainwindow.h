#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "showwindow.h"
#include "ui_showwindow.h"
#include "collectrecoverwindow.h"
#include "ui_collectrecoverwindow.h"
#include "filterwindow.h"
#include "ui_filterwindow.h"
#include "test.h"
#include "registration.h"
#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:


    void on_ShowBtn_clicked();

    void on_CollectionBtn_clicked();

    void on_FilterBtn_clicked();



    void on_pushButton_6_clicked();

private:
    Ui::MainWindow *ui;

    //ShowWindow *showwindow=new ShowWindow(this);
    //CollectRecoverWindow *collectrecoverwindow=new CollectRecoverWindow(this);
   // FilterWindow *filterwindow=new FilterWindow(this);
};

extern int window_selected;

#endif // MAINWINDOW_H





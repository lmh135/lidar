#ifndef COLLECTRECOVERWINDOW_H
#define COLLECTRECOVERWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <QFileDialog>
#include <QStringListModel>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <QtOpenGL>
#include "widget.h"
#include "point_receive.h"
#include <iostream>

using namespace std;
namespace Ui {
    class CollectRecoverWindow;
}


class CollectRecoverWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit CollectRecoverWindow(QWidget *parent = 0);
    ~CollectRecoverWindow();

    Q_point_recv_node node;
    void DrawPoints();

private:
    Ui::CollectRecoverWindow *ui;
    QString fileName;
    QStringListModel logging_model;



signals:
    void log_Updated();

public slots:
    void updateLoggingView();

private slots:
    void on_ReturnBtn_clicked();
    void on_openfileBtn_clicked();
    void on_runCore_offline_Btn_clicked();
    void on_runCore_online_Btn_clicked();
    void on_getDataBtn_clicked();
    void on_posDetectBtn_clicked();
    void on_eulerDetectBtn_clicked();
    void on_getPointBtn_clicked();
    void update_GL();
    void on_point_show_Btn_clicked();
    void on_save_point_Btn_clicked();
    void on_Color_change_Btn_clicked();
    void on_connect_Btn_clicked();
};

extern pcl::PointCloud<pcl::PointXYZ> points_recv;
extern pcl::PointCloud<pcl::PointXYZ> points_recv_temp;

extern int num;
extern bool exec_flag;

extern int window_selected;


extern int ColorMode;
extern int selectMode;
extern double vorxba,voryba,vorzba;
extern float max_x,min_x,max_y,min_y,max_z,min_z;

#endif // COLLECTRECOVERWINDOW_H

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <sys/types.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <liblas/liblas.hpp>
#include <liblas/header.hpp>
#include <pcl/point_cloud.h>
#include <QKeyEvent>
#include <pcl/point_types.h>
#include <QFileDialog>
#include <vector>
#include <QProcess>
#include <QStringListModel>
#include <QKeyEvent>
#include <vector>

//#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>

#include "data_construct.h"
#include "collectionrecover.h"
#include "point_receive.h"
#include "select.h"
#include "widget.h"
#include "filtercloud.h"
#include "registration.h"
#include "contour.h"
#include "fusion.h"
#include "checkout_rpy.h"

using namespace std;
//using namespace cv;

void draw_fuc();

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool loadlasFile(const QString &fileName); // 加载文件
    void add_cloud_page();

public slots:
    void updateLoggingView();


private slots:
    void keyPressEvent(QKeyEvent *event);

    void on_action_3_triggered();

    void on_OpenBtn_triggered();

    void on_CloseTabPageBtn_triggered();

    void on_SaveBtn_triggered();

    void on_tabWidget_currentChanged(int index);

    void on_ColorModeBtn_triggered();

    void SelectPoint();

    void on_SelectBtn_triggered();

    void on_Measure_distanceBtn_triggered();

    void on_openSourceDataBtn_clicked();

    void on_connect_Btn_clicked();

    void on_OfflineRecoverBtn_clicked();

    void on_OnlineRecoverBtn_clicked();

    void on_posDetectBtn_clicked();

    void on_eulerDetectBtn_clicked();

    void on_point_show_Btn_clicked();

    void update_GL();

    void change_selected_index();

    void on_ClearBtn_triggered();

    void on_toolBox_currentChanged(int index);

    void on_filter_comboBox_currentIndexChanged(int index);

    void on_para1_spinbox_valueChanged(double arg1);

    void on_para2_spinbox_valueChanged(double arg1);

    void on_para3_spinbox_valueChanged(double arg1);

    void on_start_filt_btn_clicked();

    void show_filted_ok();

    void show_triangle_ok();

    void show_registration_ok();

    void on_show_filted_cloud_btn_clicked();

    void on_show_triangulation_btn_clicked();

    void on_triangute_Btn_clicked();

    void on_show_registration_all_btn_clicked();

    void on_continune_registrate_btn_clicked();

    void on_start_registrate_btn_clicked();

    void on_registration_combox_currentIndexChanged(int index);

    void on_pick_cloud1_combox_currentIndexChanged(const QString &arg1);

    void on_pick_cloud2_combox_currentIndexChanged(const QString &arg1);

    void on_pick_world_cloud_combox_currentIndexChanged(const QString &arg1);

    void on_pick_pic_cloud_combox_currentIndexChanged(const QString &arg1);

    void on_check_cloud1_combox_currentIndexChanged(const QString &arg1);

    void on_check_cloud2_combox_currentIndexChanged(const QString &arg1);

    void on_counterBtn_triggered();

    void on_Save_collectRecover_Btn_clicked();


    void on_optimizing_start_btn_clicked();

    void on_fusion_start_btn_clicked();


    void on_input_ori_data1_btn_clicked();

    void on_input_ori_data2_btn_clicked();


    void on_begin_checkout_btn_clicked();

    void on_cut_ori_data1_btn_clicked();

    void on_output_ori_data1_btn_clicked();

    void on_add_matched_point_btn_clicked();



    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;

    CollectionRecover collector;
    FilterCloud filter;
    Registration registration;
    Fusion_cloud_pic fusion;
    Checkout_rpy checkout_rpy;
    ifstream ifs;

    bool collecting_flag;

    QStringListModel logging_model;

signals:
    void log_Updated();
    void start_filt();
    void start_triangulate();
    void start_registrate();
    void continue_registrate();

};




extern bool select_mode;
extern int tab_current_index;
extern vector<MYCloud> cloud_seq;
extern bool exec_flag;
extern pcl::PointCloud<pcl::PointXYZ> points_recv;
extern Filter_Para filter_para;
extern Registration_Para registration_para;
extern Checkout_Para checkout_para;

extern Contour allcontourlines;

#endif // MAINWINDOW_H

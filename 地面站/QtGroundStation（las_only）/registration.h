#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <QMainWindow>

#include <pcl/octree/octree_search.h>
#include <liblas/liblas.hpp>
#include <liblas/header.hpp>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <string>
#include<stdlib.h>
#include <unistd.h>

#include "widget.h"

using namespace std;

namespace Ui {
class registration;
}

class registration : public QMainWindow
{
    Q_OBJECT

public:
    explicit registration(QWidget *parent = 0);
    ~registration();

    bool loadlasFile(const QString &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud, int seq_num);

    void Drawcloud1();
    void Drawcloud2();
    void Drawcloud3();
    void print4x4Matrix (const Eigen::Matrix4d & matrix);

private slots:


    void on_return_Btn_clicked();

    void on_input1_Btn_clicked();

    void on_input2_Btn_clicked();
    void keyPressEvent(QKeyEvent *event);

    void SelectPoint();


    void on_switch_Btn_clicked();


    void on_pick_Btn_clicked();

    void on_switch1_Btn_clicked();

    void on_switch2_Btn_clicked();

    void on_start_Btn_clicked();

    void on_continune_Btn_clicked();

    void on_show_all_Btn_clicked();

    void on_save_Btn_clicked();

private:
    Ui::registration *ui;

    ifstream ifs;

    void draw_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud, int seq_num);
    void draw_selected_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud, int seq_num);

};

extern int window_selected;
extern int selectMode;







#endif // REGISTRATION_H

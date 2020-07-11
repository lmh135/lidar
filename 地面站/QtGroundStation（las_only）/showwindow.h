#ifndef SHOWWINDOW_H
#define SHOWWINDOW_H

#include <QMainWindow>
#include <stdio.h>
#include <QLineEdit>
//#include <widget.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <string>
#include<stdlib.h>
#include <unistd.h>
#include <math.h>
#include <QMessageBox>
#include <QTextStream>
#include <QFile>
#include <QDebug>
#include <QtOpenGL>
#include "widget.h"
#include <vector>
#include <qgl.h>
#include <QFileDialog>

#include <pcl/octree/octree_search.h>
#include <liblas/liblas.hpp>
#include <liblas/header.hpp>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <QKeyEvent>
#include "select.h"
#include "data_construct.h"

using namespace std;

namespace Ui {
    class ShowWindow;
}

class ShowWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit ShowWindow(QWidget *parent = 0);
    ~ShowWindow();
    bool loadlasFile(const QString &fileName); // 加载文件
    void DrawLasPoints();



private:
    Ui::ShowWindow *ui;
    QString fileName;
    ifstream ifs;



signals:

public slots:
    void ChangeColorMode();
   // void ChangeSelectMode();
    void ClearMode();
    //void Show_Selected_Point();
    void SelectPoint();


private slots:
    void on_ReturnBtn_clicked();
    void on_openBtn_clicked();
    void on_SelectBtn_clicked();
    void keyPressEvent(QKeyEvent *event);
    void on_SaveBtn_clicked();
    void on_Measure_distance_clicked();
};



extern int window_selected;

extern vector<int> selected_index;
extern vector<Point> selected_point_save;
extern int ColorMode;
extern int selectMode;
extern double vorxba,voryba,vorzba;
extern float max_x,min_x,max_y,min_y,max_z,min_z;

#endif // SHOWWINDOW_H

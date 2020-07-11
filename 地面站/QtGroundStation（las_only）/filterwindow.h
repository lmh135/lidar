#ifndef FILTERWINDOW_H
#define FILTERWINDOW_H

#include <QMainWindow>
#include <sstream>
#include <iostream>
#include <fstream>
#include "triangulate.h"
#include "points_array.h"
#include <QLineEdit>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <liblas/liblas.hpp>
#include <liblas/header.hpp>

#include "data_construct.h"


#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/filters/bilateral.h>
//#include <pcl/filters/fast_bilateral.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <armadillo>


#include <contour.h>

using namespace std;
using namespace arma;

namespace Ui{
    class FilterWindow;
}

class FilterWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit FilterWindow(QWidget *parent = 0);
    ~FilterWindow();

    void Draw_unfilted_Points();
    void Draw_filted_Points();
    void Draw_triangulations();

private:
    Ui::FilterWindow *ui;
    bool loadlasFile(const QString &fileName); // 加载文件

signals:

public slots:
private slots:
    void on_RetuenBtn_clicked();
    void on_input_Btn_clicked();
    void on_save_Btn_clicked();
    void on_DEM_Btn_clicked();
    void on_smooth_Btn_clicked();
    void on_curvature_Btn_clicked();
    void on_normal_Btn_clicked();
    void on_bothside_Btn_clicked();
    void on_thick_Btn_clicked();
    void on_voxelgrid_Btn_clicked();
    void on_statistics_Btn_clicked();
    void on_conture_Btn_clicked();
};

extern int window_selected;
extern int ColorMode;
extern int selectMode;
extern double vorxba,voryba,vorzba;
extern float max_x,min_x,max_y,min_y,max_z,min_z;

#endif // FILTERWINDOW_H

#ifndef FILTERCLOUD_H
#define FILTERCLOUD_H

#include <QObject>
#include <QMessageBox>

#include <iostream>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/bilateral.h>
//#include <pcl/filters/fast_bilateral.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <armadillo>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <math.h>

#include "triangulate.h"
#include "points_array.h"
#include "contour.h"
#include "data_construct.h"

using namespace std;
using namespace arma;

class FilterCloud : public QObject
{
    Q_OBJECT
public:
    explicit FilterCloud(QObject *parent = nullptr);

signals:
    void filter_ok();
    void triangulate_ok();

public slots:
    void start_filter();
    void las2triangulation();

private:
    void smooth_Filter(float grid_size);
    void curvature_Filter(int K_num);
    void normal_Filter(int K_num, float alpha);
    void bothside_Filter(int K_num);
    void thick_Filter(float max_thick, float min_thick, float selected_thick);
    void voxelgrid_Filter(float grid_size);

};

extern Filter_Para filter_para;
extern vector<MYCloud> cloud_seq;
extern int tab_current_index;

#endif // FILTERCLOUD_H

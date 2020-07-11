#ifndef FUSION_H
#define FUSION_H

#include <QObject>
#include <QMessageBox>

#include <iostream>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <armadillo>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <math.h>
#include <vector>

#include "data_construct.h"

#define DEG2RAD 0.01745329251

using namespace std;
using namespace arma;


class Fusion_cloud_pic : public QObject
{
    Q_OBJECT
public:
    explicit Fusion_cloud_pic(QObject *parent = nullptr);



    void init_pic_cloud_selected(pcl::PointCloud<pcl::PointXYZ> pic_cloud_selected);
    void init_world_cloud_selected(pcl::PointCloud<pcl::PointXYZ> world_cloud_selected);
    void init_pic_cloud_whole(pcl::PointCloud<pcl::PointXYZ> pic_cloud_whole, std::vector<Color>& color);
    void init_world_cloud_whole(pcl::PointCloud<pcl::PointXYZ> world_cloud_whole);


    //输入:初始变换参数，各参数步长，各参数允许误差，最大步数，最大变步长数
    //输出:优化参数，优化最短距离和，计算次数总和
    Search_result One_D_Search(Trans_para7 init_para,double step_length[],double err_allow[],int max_step_num,int max_motified_step_num);

    //输入:初始变换参数，各参数步长,各参数允许误差，最大轮次
    //输出:优化参数，优化最短距离和
    Search_result Powell_Search(Trans_para7 init_para,double init_step_length[],double err_allow_whole[],int max_ring_time);

    Trans_para7 calcu_init_para(Trans_para7 init_para, int num);

    void Find_color(Trans_para7 para);
private:

    double calcu_sum_distance(Trans_para7 para);



    void generateCbn(double roll,double pitch,double yaw);

    double Cbn[3][3];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pic_selected;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world_selected;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pic_whole;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world_whole;

    std::vector<Color> pic_color;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_world;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_pic;




};

extern Fusion_Para fusion_para;

extern vector<MYCloud> cloud_seq;
extern int tab_current_index;





#endif // FUSION_H

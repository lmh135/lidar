#ifndef CHECKOUT_RPY_H
#define CHECKOUT_RPY_H

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

#include<unistd.h>
#include<stdlib.h>
#include<sys/types.h>
#include<stdio.h>
#include<netinet/in.h>
#include<arpa/inet.h>

#include "data_construct.h"


#define DEG2RAD 0.01745329251

using namespace std;
using namespace arma;


class Checkout_rpy : public QObject
{
    Q_OBJECT
public:
    explicit Checkout_rpy(QObject *parent = nullptr);

    QString binary_file_cloud1;
    QString binary_file_cloud2;

    vector <Ori_data> ori_cloud1;
    vector <Ori_data> ori_cloud2;

    //输入:初始变换参数，各参数步长，各参数允许误差，最大步数，最大变步长数
    //输出:优化参数
    Trans_para3 One_D_Search(Trans_para3 init_para,double step_length[],double err_allow[],int max_step_num,int max_motified_step_num);

    //输入:初始变换参数，各参数步长,各参数允许误差，最大轮次
    //输出:优化参数
    Trans_para3 Powell_Search(Trans_para3 init_para,double init_step_length[],double err_allow_whole[],int max_ring_time);


private:
    void generateCbn(double roll,double pitch,double yaw);

    double calcu_sum_distance(Trans_para3 para);

    void calcu_lidar_point(vector <Ori_data> &tdata, Trans_para3 para, pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud);

    double Cbn[3][3];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;



};

























#endif // CHECKOUT_RPY_H

#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <QObject>
#include <QString>
#include <QMessageBox>

#include<stdlib.h>
#include <unistd.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>

#include "data_construct.h"

#define DEG2RAD 0.01745329251

using namespace std;

class Registration : public QObject
{
    Q_OBJECT
public:
    explicit Registration(QObject *parent = nullptr);

    void print4x4Matrix (const Eigen::Matrix4d & matrix);

    //输入:初始变换参数，各参数步长，各参数允许误差，最大步数，最大变步长数
    //输出:优化参数
    Trans_para6 One_D_Search(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud,\
                             Trans_para6 init_para,double step_length[],double err_allow[],int max_step_num,int max_motified_step_num);

    Trans_para6 Powell_Search(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud,\
                              Trans_para6 init_para,double init_step_length[],double err_allow_whole[],int max_ring_time);

    void MY_icp_search(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud);

signals:
    void registration_ok();
    void continue_step_ok();

public slots:
    void start_registration();
    void continue_registration();

private:
    void classical_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud);

    double calcu_sum_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud,Trans_para6 para);

    void generateCbn(double roll,double pitch,double yaw);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;

    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;

    double Cbn[3][3];

};

extern int tab_current_index;

extern vector<MYCloud> cloud_seq;

extern Registration_Para registration_para;

#endif // REGISTRATION_H

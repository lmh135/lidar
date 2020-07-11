#ifndef DATA_CONSTRUCT_H
#define DATA_CONSTRUCT_H

#define share_num 50
#include <vector>
#include <pcl/point_cloud.h>
#include <QString>
using namespace std;

typedef struct _Point
{
    double x;
    double y;
    double z;
}Point;

typedef struct _Color
{
    float r;
    float g;
    float b;
}Color;

typedef struct _Triangle{
    Point p[3];
}Triangle;


typedef struct _Points
{
    Point p[share_num][192];
    int id_num;
    int flag;
}Points;

typedef struct _Ori_Data
{
    long int id_num;
    float euler[3];
    double posNED[3];
    Point p_laser;
}Ori_data;

typedef struct _Trans_para6{
    Point P0;
    double roll;
    double pitch;
    double yaw;
}Trans_para6;

typedef struct _Trans_para7{
    Point P0;
    double roll;
    double pitch;
    double yaw;
    double u;
}Trans_para7;


typedef struct _Trans_para3{
    double dr,dp,dy;
}Trans_para3;


typedef struct _MYCloud{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_filted;
    pcl::PointCloud<pcl::PointXYZ> cloud_registrated1;
    pcl::PointCloud<pcl::PointXYZ> cloud_registrated2;
    vector <Triangle> cloud_triangles;
    vector <Triangle> cloud_filted_triangles;
    vector <int> selected_index;
    vector <Color> color;
    bool isColor;
    float min[3];
    float max[3];
    float min_filted[3];
    float max_filted[3];
    float min_registrated[3];
    float max_registrated[3];
    float s_ruler_point[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    int show_index;
    QString mycloud_name;
}MYCloud;

typedef struct _Filter_Para{
    int KNear;
    float NormalDirectionAdjust;
    float GridSize;
    float max_thick;
    float min_thick;
    float selected_thick;
    int filter_index;
}Filter_Para;

typedef struct _Registration_Para{
    int registration_index;
    QString cloud1_name;
    QString cloud2_name;
    Eigen::Matrix4d transformation_matrix;
    Trans_para6 para6;
}Registration_Para;




typedef struct _Search_result{
    Trans_para7 para;
    double optimized_distance;
    unsigned long time;
}Search_result;

typedef struct _Fusion_Para{
    int fusion_index;
    Trans_para7 trans_para7;
    QString world_cloud_name;
    QString pic_cloud_name;
    vector <Point> world_point;
    vector <Point> pic_point;

}Fusion_Para;

typedef struct _Checkout_Para{
   // int checkout_index;
    QString cloud1_name;
    QString cloud2_name;
    Trans_para3 trans_para3;
}Checkout_Para;

#endif // DATA_CONSTRUCT_H

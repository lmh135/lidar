#ifndef SELECT_H
#define SELECT_H

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <QGLWidget>
#include <iostream>
#include <stdlib.h>
#include <vector>
using namespace std;


int DoOpenGLSelection(float start_x, float start_y, float current_x, float current_y, pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud,double vorxba,double voryba,double vorzba);


void Selection_init(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud);
void Selection_destroy(void);
vector<int> GetSelectedIndex(unsigned int hits);


//extern double vorxba,voryba,vorzba;



#endif // SELECT_H

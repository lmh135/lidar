#ifndef POINT_RECEIVE_H
#define POINT_RECEIVE_H
#include <QThread>
#include<iostream>

#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include "shm.h"
#include "data_construct.h"

using namespace std;

class Q_point_recv_node : public QThread {
    Q_OBJECT
    public:
        Q_point_recv_node();
        virtual ~Q_point_recv_node();

        bool init();
        void run();

    signals:
        void update_Point();
        void recv_index_changed();

    private:
};





extern bool exec_flag;
extern pcl::PointCloud<pcl::PointXYZ> points_recv;
//extern pcl::PointCloud<pcl::PointXYZ> points_recv_temp;


extern vector<MYCloud> cloud_seq;
extern int tab_current_index;








#endif // POINT_RECEIVE_H

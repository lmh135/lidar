/#include <liblas/liblas.hpp>
//#include <iomanip>
//#include <sstream>
//#include <fstream>  // std::ofstream
//#include <iostream>
//#include <string>
//#include <stdio.h>
//#include <stdlib.h>
//#include <liblas/header.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/octree/octree.h>
//using namespace std;

//#define WIDTH  0.2

//typedef struct _POINT
//{
//    double x;
//    double y;
//    double z;
//}Point;

//int main()
//{
//    srand ((unsigned int) time (NULL));

//    ifstream ifs;
//        ifs.open("0414ftj6.las", ios::in | ios::binary);
//    if (ifs == NULL)
//    {
//            cout<<"File Error!"<<endl;
//            return 0;
//        }
//        liblas::ReaderFactory f ;
//    liblas::Reader reader = f.CreateWithStream(ifs);
//        liblas::Header const& header = reader.GetHeader();

//    int sum_point_num = header.GetPointRecordsCount();

//    cout<<"Number of point records     : "<<sum_point_num<<endl;          //记录的点数信息,下面的信息比较重要，都列出来了，具体意思也比较好理解

//        cout<<setprecision(9)<<"X scale factor              : "<<header.GetScaleX()<<endl;
//        cout<<"Y scale factor              : "<<header.GetScaleY()<<endl;
//        cout<<"Z scale factor              : "<<header.GetScaleZ()<<endl;
//        cout<<"X offset                    : "<<header.GetOffsetX()<<endl;
//        cout<<"Y offset                    : "<<header.GetOffsetY()<<endl;
//        cout<<"Z offset                    : "<<header.GetOffsetZ()<<endl;
//        cout<<"Max X                       : "<<header.GetMaxX()<<endl;
//        cout<<"Max Y                       : "<<header.GetMaxY()<<endl;
//        cout<<"Max Z                       : "<<header.GetMaxZ()<<endl;
//        cout<<"Min X                       : "<<header.GetMinX()<<endl;
//        cout<<"Min Y                       : "<<header.GetMinY()<<endl;
//    cout<<"Min Z                       : "<<header.GetMinZ()<<endl;

//    double max_x = header.GetMaxX()+0.05;
//    double max_y = header.GetMaxY()+0.05;
//    double max_z = header.GetMaxZ()+0.05;
//    double min_x = header.GetMinX()-0.05;
//    double min_y = header.GetMinY()-0.05;
//    double min_z = header.GetMinZ()-0.05;

//    int num_x_block = floor((max_x-min_x)/WIDTH);
//    int num_y_block = floor((max_y-min_y)/WIDTH);



//    printf("num_block: %d %d\n",num_x_block,num_y_block);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

//    int i = 0;
//    cloud_in->width = sum_point_num;
//    cloud_in->height = 1;
//    cloud_in->points.resize (cloud_in->width * cloud_in->height);

//    while(reader.ReadNextPoint())
//    {
//        //reader.ReadPointAt(2);

//        cloud_in->points[i].x = reader.GetPoint().GetX();
//        cloud_in->points[i].y = reader.GetPoint().GetY();
//        cloud_in->points[i].z = reader.GetPoint().GetZ();

//        i++;

//    }

//    float resolution = WIDTH;

//    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

//    octree.defineBoundingBox(min_x,min_y,min_z,max_x,max_y,max_z);

//    octree.setInputCloud(cloud_in);
//    octree.addPointsFromInputCloud();

//    pcl::PointXYZ searchPoint;
//    //reader.ReadPointAt(10000);

//    searchPoint.x = min_x;
//    searchPoint.y = min_y;
//    searchPoint.z = min_z;

//    for(searchPoint.x = min_x;searchPoint.x<=max_x;searchPoint.x+=WIDTH)
//    {
//        for(searchPoint.y = min_y;searchPoint.y<=max_y;searchPoint.y+=WIDTH)
//        {
//            double temp_min_z = 100;
//            std::vector<Point> temp_point_vec;

//            for(searchPoint.z = min_z;searchPoint.z<=max_z;searchPoint.z+=WIDTH)
//            {
//                std::vector<int> pointIdxVec;
//                if (octree.voxelSearch (searchPoint, pointIdxVec))
//                {
//                    /*
//                    std::cout << "Neighbors within voxel search at (" << searchPoint.x
//                            << " " << searchPoint.y
//                            << " " << searchPoint.z << ")"
//                            << std::endl;

//                        for (size_t i = 0; i < pointIdxVec.size (); ++i)
//                        std::cout << "    " << cloud_in->points[pointIdxVec[i]].x
//                                << " " << cloud_in->points[pointIdxVec[i]].y
//                                << " " << cloud_in->points[pointIdxVec[i]].z << std::endl;
//                    */

//                    for (size_t i = 0; i < pointIdxVec.size (); ++i)
//                    {
//                        Point p;
//                        p.x = cloud_in->points[pointIdxVec[i]].x;
//                        p.y = cloud_in->points[pointIdxVec[i]].y;
//                        p.z = cloud_in->points[pointIdxVec[i]].z;
//                        temp_point_vec.push_back(p);
//                        if(p.z < temp_min_z)
//                            temp_min_z = p.z;
//                    }

//                }
//            }

//            vector<Point>::iterator item;
//            for(item=temp_point_vec.begin(); item!=temp_point_vec.end(); item++)
//            {
//                if(item->z < temp_min_z+0.1)
//                {

//                    pcl::PointXYZ point_selected;
//                    point_selected.x = item->x;
//                    point_selected.y = item->y;
//                    point_selected.z = item->z;

//                    cloud_out->points.push_back(point_selected);

//                }
//            }

//        }
//    }
//    printf("ok! \n");

//    cloud_out->width = cloud_out->points.size();
//    cloud_out->height = 1;

//    FILE* p1=NULL;
//    p1 = fopen("0416ftj2.txt","a+");


//    for (size_t i=0; i< cloud_out->points.size (); ++i)
//    {
//        fprintf(p1,"%lf %lf %lf \n",cloud_out->points[i].x,cloud_out->points[i].y,cloud_out->points[i].z);
//    }
//    fclose(p1);
//    return 0;
//}


















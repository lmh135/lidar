#include "filtercloud.h"

FilterCloud::FilterCloud(QObject *parent) : QObject(parent)
{

}


void FilterCloud::start_filter()
{
    if(cloud_seq.empty())
    {
        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"错误","还未输入点云!");
        delete temp;
        return;
    }
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if((*item_cloud).selected_index.empty())
    {
        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"错误","还未选择点云!");
        delete temp;
        return;
    }

    int filter_index = filter_para.filter_index;
    int k_num;
    float alpha,max_h,min_h,selected_h,size;
    switch(filter_index)
    {
    case 1:
        k_num = floor(filter_para.KNear);
        curvature_Filter(k_num);
        break;
    case 2:
        k_num = floor(filter_para.KNear);
        alpha = filter_para.NormalDirectionAdjust;
        normal_Filter(k_num, alpha);
        break;
    case 3:
        max_h = filter_para.max_thick;
        min_h = filter_para.min_thick;
        selected_h = filter_para.selected_thick;
        thick_Filter(max_h, min_h, selected_h);
        break;
    case 4:
        size = filter_para.GridSize;
        voxelgrid_Filter(size);
        break;
    case 5:
        k_num = floor(filter_para.KNear);
        bothside_Filter(k_num);
        break;
    case 6:
        size = filter_para.GridSize;
        smooth_Filter(size);
        break;
    default:
        break;
    }


}


void FilterCloud::voxelgrid_Filter(float grid_size)
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;


    pcl::PointCloud<pcl::PointXYZ> cloud_unfilted;
    cloud_unfilted.points.clear();

    for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
        p.y = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
        p.z = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
        cloud_unfilted.points.push_back(p);
    }

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_unfilted.makeShared());
//    sor.setLeafSize(0.3f,0.3f,0.3f);
    sor.setLeafSize(grid_size,grid_size,grid_size);
    sor.filter((*item_cloud).cloud_filted);

    float min_x = 100000;
    float min_y = 100000;
    float min_z = 100000;
    float max_x = -100000;
    float max_y = -100000;
    float max_z = -100000;

    for(unsigned int i=0; i<(*item_cloud).cloud_filted.points.size(); i++)
    {
        float x = (*item_cloud).cloud_filted.points[i].x;
        float y = (*item_cloud).cloud_filted.points[i].y;
        float z = (*item_cloud).cloud_filted.points[i].z;
        min_x = min_x < x ? min_x : x;
        min_y = min_y < y ? min_y : y;
        min_z = min_z < z ? min_z : z;
        max_x = max_x > x ? max_x : x;
        max_y = max_y > y ? max_y : y;
        max_z = max_z > z ? max_z : z;
    }

    (*item_cloud).min_filted[0] = min_x;
    (*item_cloud).min_filted[1] = min_y;
    (*item_cloud).min_filted[2] = min_z;
    (*item_cloud).max_filted[0] = max_x;
    (*item_cloud).max_filted[1] = max_y;
    (*item_cloud).max_filted[2] = max_z;

    QWidget *temp = new QWidget;
    QMessageBox::information(temp,"完成","栅格滤波完成!");
    delete temp;

    emit filter_ok();
}
void FilterCloud::smooth_Filter(float grid_size)
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;


    pcl::PointCloud<pcl::PointXYZ> cloud_unfilted;
    cloud_unfilted.points.clear();

    float min_x = 100000;
    float min_y = 100000;
    float min_z = 100000;
    float max_x = -100000;
    float max_y = -100000;
    float max_z = -100000;

    for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
        p.y = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
        p.z = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
        cloud_unfilted.points.push_back(p);
        min_x = min_x < p.x ? min_x : p.x;
        min_y = min_y < p.y ? min_y : p.y;
        min_z = min_z < p.z ? min_z : p.z;
        max_x = max_x > p.x ? max_x : p.x;
        max_y = max_y > p.y ? max_y : p.y;
        max_z = max_z > p.z ? max_z : p.z;
    }



    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (grid_size);

    octree.defineBoundingBox(min_x-0.05,min_y-0.05,min_z-0.05,max_x+0.05,max_y+0.05,max_z+0.05);
    octree.setInputCloud(cloud_unfilted.makeShared());
    octree.addPointsFromInputCloud();

    pcl::PointXYZ searchPoint;
    searchPoint.x = min_x;
    searchPoint.y = min_y;
    searchPoint.z = min_z;

    for(searchPoint.x = min_x;searchPoint.x<=max_x;searchPoint.x+=grid_size)
    {
        for(searchPoint.y = min_y;searchPoint.y<=max_y;searchPoint.y+=grid_size)
        {
            double temp_min_z = 10000;
            vector<Point> temp_point_vec;

            for(searchPoint.z = min_z;searchPoint.z<=max_z;searchPoint.z+=grid_size)
            {
                vector<int> pointIdxVec;
                if (octree.voxelSearch (searchPoint, pointIdxVec))
                {
                    for (size_t i = 0; i < pointIdxVec.size (); ++i)
                    {
                        Point p;
                        p.x = cloud_unfilted.points[pointIdxVec[i]].x;
                        p.y = cloud_unfilted.points[pointIdxVec[i]].y;
                        p.z = cloud_unfilted.points[pointIdxVec[i]].z;
                        temp_point_vec.push_back(p);
                        if(p.z < temp_min_z)
                            temp_min_z = p.z;
                    }
                }
            }

            if(!temp_point_vec.empty())
            {
                vector<Point>::iterator item;
                for(item=temp_point_vec.begin(); item!=temp_point_vec.end(); item++)
                {
                    if(item->z < temp_min_z+0.02)
                    {
                        pcl::PointXYZ point_selected;
                        point_selected.x = item->x;
                        point_selected.y = item->y;
                        point_selected.z = item->z;

                        (*item_cloud).cloud_filted.points.push_back(point_selected);

                    }
                }
            }
        }
    }

    float min_x2 = 100000;
    float min_y2 = 100000;
    float min_z2 = 100000;
    float max_x2 = -100000;
    float max_y2 = -100000;
    float max_z2 = -100000;

    for(unsigned int i=0; i<(*item_cloud).cloud_filted.points.size(); i++)
    {
        float x = (*item_cloud).cloud_filted.points[i].x;
        float y = (*item_cloud).cloud_filted.points[i].y;
        float z = (*item_cloud).cloud_filted.points[i].z;
        min_x2 = min_x2 < x ? min_x2 : x;
        min_y2 = min_y2 < y ? min_y2 : y;
        min_z2 = min_z2 < z ? min_z2 : z;
        max_x2 = max_x2 > x ? max_x2 : x;
        max_y2 = max_y2 > y ? max_y2 : y;
        max_z2 = max_z2 > z ? max_z2 : z;
    }

    (*item_cloud).min_filted[0] = min_x2;
    (*item_cloud).min_filted[1] = min_y2;
    (*item_cloud).min_filted[2] = min_z2;
    (*item_cloud).max_filted[0] = max_x2;
    (*item_cloud).max_filted[1] = max_y2;
    (*item_cloud).max_filted[2] = max_z2;

    QWidget *temp = new QWidget;
    QMessageBox::information(temp,"完成","平滑滤波完成!");
    delete temp;

    emit filter_ok();
}
void FilterCloud::curvature_Filter(int K_num)
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;


    pcl::PointCloud<pcl::PointXYZ> cloud_unfilted;
    cloud_unfilted.points.clear();

    float min_x = 100000;
    float min_y = 100000;
    float min_z = 100000;
    float max_x = -100000;
    float max_y = -100000;
    float max_z = -100000;

    for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
        p.y = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
        p.z = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
        cloud_unfilted.points.push_back(p);
        min_x = min_x < p.x ? min_x : p.x;
        min_y = min_y < p.y ? min_y : p.y;
        min_z = min_z < p.z ? min_z : p.z;
        max_x = max_x > p.x ? max_x : p.x;
        max_y = max_y > p.y ? max_y : p.y;
        max_z = max_z > p.z ? max_z : p.z;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_RadFilted;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);//点和法向量


    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_unfilted.makeShared());
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius(0);
    outrem.filter(cloud_RadFilted);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;    //法向计算
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_RadFilted.makeShared());
    n.setInputCloud (cloud_RadFilted.makeShared());
    n.setSearchMethod (tree);
    n.setKSearch(K_num);
    n.compute (*normals);
    pcl::concatenateFields(cloud_RadFilted, *normals, *cloud_with_normals);

    float WIDTH = 0.3;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (WIDTH);
    octree.defineBoundingBox(min_x-0.05,min_y-0.05,min_z-0.05,max_x+0.05,max_y+0.05,max_z+0.05);
    octree.setInputCloud(cloud_RadFilted.makeShared());
    octree.addPointsFromInputCloud();
    pcl::PointXYZ searchPoint;
    searchPoint.x = min_x;
    searchPoint.y = min_y;
    searchPoint.z = min_z;
    int VoxelGrid=0;//栅格数量
    float Svari = 0;//全体点云的方差Svari

    for(searchPoint.x = min_x;searchPoint.x<=max_x;searchPoint.x+=WIDTH)    //计算全体的方差Svari
    {
        for(searchPoint.y = min_y;searchPoint.y<=max_y;searchPoint.y+=WIDTH)
        {
            for(searchPoint.z = min_z;searchPoint.z<=max_z;searchPoint.z+=WIDTH)
            {
                std::vector<int> pointIdxVec;
                float average_cur = 0;//每个栅格的平均曲率
                if (octree.voxelSearch (searchPoint, pointIdxVec))
                {
                    float vari=0;
                    VoxelGrid++;
                    for (size_t i = 0; i < pointIdxVec.size (); ++i)
                    {
                        average_cur+=cloud_with_normals->points[pointIdxVec[i]].curvature;//每个点的曲率：cloud_with_normals->points[pointIdxVec[i]].curvature
                    }
                    average_cur=average_cur/pointIdxVec.size ();

                    for (size_t i = 0; i < pointIdxVec.size (); ++i)
                    {
                        vari+=(cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur)*(cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur);
                    }
                    vari=vari/pointIdxVec.size();
                    Svari+=vari;
                }
            }
        }
    }

    Svari=Svari/VoxelGrid;

    for(searchPoint.x = min_x;searchPoint.x<=max_x;searchPoint.x+=WIDTH)
    {
        for(searchPoint.y = min_y;searchPoint.y<=max_y;searchPoint.y+=WIDTH)
        {

            for(searchPoint.z = min_z;searchPoint.z<=max_z;searchPoint.z+=WIDTH)
            {
                std::vector<int> pointIdxVec;
                float average_cur = 0;
                if (octree.voxelSearch (searchPoint, pointIdxVec))
                {
                    float vari=0;
                    for (size_t i = 0; i < pointIdxVec.size (); ++i)
                    {
                            average_cur+=cloud_with_normals->points[pointIdxVec[i]].curvature;
                    }
                    average_cur=average_cur/pointIdxVec.size ();

                    for (size_t i = 0; i < pointIdxVec.size (); ++i)
                    {
                        vari+=(cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur)*(cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur);
                    }

                    vari=vari/pointIdxVec.size();                    //每个栅格的方差vari
                    if(vari < Svari)
                    {
                         long idsel=0;
                         for (size_t i = 0; i < pointIdxVec.size (); ++i)
                         {
                            double temp_distance=0;
                            double min_distance = 100;
                            temp_distance=(cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur)*(cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur);
                            if(temp_distance<min_distance)
                            {
                                min_distance = temp_distance;
                                idsel=pointIdxVec[i];
                            }
                          }
                          (*item_cloud).cloud_filted.points.push_back(cloud_RadFilted.points[idsel]);
                    }

                    else if(vari >= Svari)
                    {
                        pcl::PointXYZ p;
                        for (size_t i = 0; i < pointIdxVec.size (); i++)
                        {
                            if((cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur)*
                               (cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur) >= vari)
                            {
                                p.x = cloud_RadFilted.points[pointIdxVec[i]].x;
                                p.y = cloud_RadFilted.points[pointIdxVec[i]].y;
                                p.z = cloud_RadFilted.points[pointIdxVec[i]].z;
                                (*item_cloud).cloud_filted.points.push_back(p);
                            }
                        }
                    }
                }
            }
        }
    }

    float min_x2 = 100000;
    float min_y2 = 100000;
    float min_z2 = 100000;
    float max_x2 = -100000;
    float max_y2 = -100000;
    float max_z2 = -100000;

    for(unsigned int i=0; i<(*item_cloud).cloud_filted.points.size(); i++)
    {
        float x = (*item_cloud).cloud_filted.points[i].x;
        float y = (*item_cloud).cloud_filted.points[i].y;
        float z = (*item_cloud).cloud_filted.points[i].z;
        min_x2 = min_x2 < x ? min_x2 : x;
        min_y2 = min_y2 < y ? min_y2 : y;
        min_z2 = min_z2 < z ? min_z2 : z;
        max_x2 = max_x2 > x ? max_x2 : x;
        max_y2 = max_y2 > y ? max_y2 : y;
        max_z2 = max_z2 > z ? max_z2 : z;
    }

    (*item_cloud).min_filted[0] = min_x2;
    (*item_cloud).min_filted[1] = min_y2;
    (*item_cloud).min_filted[2] = min_z2;
    (*item_cloud).max_filted[0] = max_x2;
    (*item_cloud).max_filted[0] = max_y2;
    (*item_cloud).max_filted[0] = max_z2;

    QWidget *temp = new QWidget;
    QMessageBox::information(temp,"完成","曲率滤波完成!");
    delete temp;

    emit filter_ok();

}
void FilterCloud::normal_Filter(int K_num, float alpha)
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    pcl::PointCloud<pcl::PointXYZ> cloud_unfilted;
    cloud_unfilted.points.clear();

    float min_x = 100000;
    float min_y = 100000;
    float min_z = 100000;
    float max_x = -100000;
    float max_y = -100000;
    float max_z = -100000;

    for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
        p.y = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
        p.z = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
        cloud_unfilted.points.push_back(p);
        min_x = min_x < p.x ? min_x : p.x;
        min_y = min_y < p.y ? min_y : p.y;
        min_z = min_z < p.z ? min_z : p.z;
        max_x = max_x > p.x ? max_x : p.x;
        max_y = max_y > p.y ? max_y : p.y;
        max_z = max_z > p.z ? max_z : p.z;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_RadFilted;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);//点和法向量

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_unfilted.makeShared());
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius(0);
    outrem.filter(cloud_RadFilted);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;    //法向计算
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_RadFilted.makeShared());
    n.setInputCloud (cloud_RadFilted.makeShared());
    n.setSearchMethod (tree);
    n.setKSearch(50);
    n.compute (*normals);
    pcl::concatenateFields(cloud_RadFilted, *normals, *cloud_with_normals);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_RadFilted.makeShared());
    pcl::PointXYZ searchPoint;
    pcl::PointXYZ dealPoint;
    for(int i=0;i<cloud_RadFilted.points.size();i++)
    {
        searchPoint.x = cloud_RadFilted.points[i].x;
        searchPoint.y = cloud_RadFilted.points[i].y;
        searchPoint.z = cloud_RadFilted.points[i].z;

        float norx;
        float nory;
        float norz;

        if(cloud_with_normals->points[i].normal_z > 0)
        {
            norx=-cloud_with_normals->points[i].normal_x;
            nory=-cloud_with_normals->points[i].normal_y;
            norz=-cloud_with_normals->points[i].normal_z;
        }
        else
        {
            norx=cloud_with_normals->points[i].normal_x;
            nory=cloud_with_normals->points[i].normal_y;
            norz=cloud_with_normals->points[i].normal_z;
        }

        int K = K_num;//50

        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {

          mat U(K,3);
          mat Z(K,1);
          mat ABC(3,1);

          for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
          {
              U(i,0)=cloud_RadFilted.points[pointIdxNKNSearch[i]].x;
              U(i,1)=cloud_RadFilted.points[pointIdxNKNSearch[i]].y;
              U(i,2)=1;
              Z(i,0)=cloud_RadFilted.points[pointIdxNKNSearch[i]].z;
          }
          //ABC=inv( ( U.t() )*U )* ( U.t() )*Z;
          ABC=solve(U,Z);
          //ABC.print("ABC:");

          float dealnorx;
          float dealnory;
          float dealnorz;

          double dNormalLength=sqrt(ABC(0,0)*ABC(0,0)+ABC(1,0)*ABC(1,0)+1);
          if(dNormalLength!=0.0)
          {
              dealnorx=ABC(0,0)/dNormalLength;
              dealnory=ABC(1,0)/dNormalLength;
              dealnorz=-1/dNormalLength;
          }
          else
          {
              dealnorx=0.0;
              dealnory=0.0;
              dealnorz=0.0;
          }

          if(dealnorz > 0)
          {
              dealnorx=-dealnorx;
              dealnory=-dealnory;
              dealnorz=-dealnorz;
          }

//          cout<<"norxyz:"<<norx<<" "<<nory<<" "<<norz<<" dealnorxyz:"<<dealnorx<<" "<<dealnory<<" "<<dealnorz<<endl;

//          float avenorx=( norx+dealnorx )/2;
//          float avenory=( nory+dealnory )/2;
//          float avenorz=( norz+dealnorz )/2;
//          float avenorx=dealnorx;
//          float avenory=dealnory;
//          float avenorz=dealnorz;

            float avenorx=dealnorx*alpha+ norx*(1-alpha);
            float avenory=dealnory*alpha+ nory*(1-alpha);
            float avenorz=dealnorz*alpha+ norz*(1-alpha);

          dealPoint.z=( -ABC(0,0)*avenorx*searchPoint.z - ABC(1,0)*avenory*searchPoint.z + ABC(2,0)*avenorz + ABC(0,0)*searchPoint.x*avenorz + ABC(1,0)*searchPoint.y*avenorz)/( avenorz - ABC(0,0)*avenorx  - ABC(1,0)*avenory ) ;
          dealPoint.x=( dealPoint.z - searchPoint.z )*avenorx/avenorz + searchPoint.x;
          dealPoint.y=( dealPoint.z - searchPoint.z )*avenory/avenorz + searchPoint.y;

          (*item_cloud).cloud_filted.points.push_back(dealPoint);
        }
    }

    float min_x2 = 100000;
    float min_y2 = 100000;
    float min_z2 = 100000;
    float max_x2 = -100000;
    float max_y2 = -100000;
    float max_z2 = -100000;

    for(unsigned int i=0; i<(*item_cloud).cloud_filted.points.size(); i++)
    {
        float x = (*item_cloud).cloud_filted.points[i].x;
        float y = (*item_cloud).cloud_filted.points[i].y;
        float z = (*item_cloud).cloud_filted.points[i].z;
        min_x2 = min_x2 < x ? min_x2 : x;
        min_y2 = min_y2 < y ? min_y2 : y;
        min_z2 = min_z2 < z ? min_z2 : z;
        max_x2 = max_x2 > x ? max_x2 : x;
        max_y2 = max_y2 > y ? max_y2 : y;
        max_z2 = max_z2 > z ? max_z2 : z;
    }

    (*item_cloud).min_filted[0] = min_x2;
    (*item_cloud).min_filted[1] = min_y2;
    (*item_cloud).min_filted[2] = min_z2;
    (*item_cloud).max_filted[0] = max_x2;
    (*item_cloud).max_filted[0] = max_y2;
    (*item_cloud).max_filted[0] = max_z2;

    QWidget *temp = new QWidget;
    QMessageBox::information(temp,"完成","法向滤波完成!");
    delete temp;

    emit filter_ok();

}
void FilterCloud::bothside_Filter(int K_num)
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    pcl::PointCloud<pcl::PointXYZ> cloud_unfilted;
    cloud_unfilted.points.clear();

    float min_x = 100000;
    float min_y = 100000;
    float min_z = 100000;
    float max_x = -100000;
    float max_y = -100000;
    float max_z = -100000;

    for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
        p.y = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
        p.z = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
        cloud_unfilted.points.push_back(p);
        min_x = min_x < p.x ? min_x : p.x;
        min_y = min_y < p.y ? min_y : p.y;
        min_z = min_z < p.z ? min_z : p.z;
        max_x = max_x > p.x ? max_x : p.x;
        max_y = max_y > p.y ? max_y : p.y;
        max_z = max_z > p.z ? max_z : p.z;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud_RadFilted;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);//点和法向量

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_unfilted.makeShared());
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius(1);

    outrem.filter((*item_cloud).cloud_filted);
//    outrem.filter(cloud_RadFilted);

//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;    //法向计算
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud(cloud_RadFilted.makeShared());
//    n.setInputCloud (cloud_RadFilted.makeShared());
//    n.setSearchMethod (tree);
//    n.setKSearch(K_num);
//    n.compute (*normals);
//    pcl::concatenateFields(cloud_RadFilted, *normals, *cloud_with_normals);

//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    kdtree.setInputCloud (cloud_RadFilted.makeShared());
//    pcl::PointXYZ searchPoint;
//    float sigmac=2;
//    float sigmas=0.8;
//    for(int i=0;i<cloud_RadFilted.points.size();i++)
//    {
//        searchPoint.x = cloud_RadFilted.points[i].x;
//        searchPoint.y = cloud_RadFilted.points[i].y;
//        searchPoint.z = cloud_RadFilted.points[i].z;

//        float norx=cloud_with_normals->points[i].normal_x;
//        float nory=cloud_with_normals->points[i].normal_y;
//        float norz=cloud_with_normals->points[i].normal_z;
//        float cur=cloud_with_normals->points[i].curvature;

//        int K = 50;
//        std::vector<int> pointIdxNKNSearch(K);
//        std::vector<float> pointNKNSquaredDistance(K);

//        float snum=0;
//        float sden=0;
//        float d=0;
//        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//        {

//          for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
//          {
//              float dis= sqrt(pointNKNSquaredDistance[i]);
//              float wsigmac=exp( -(dis*dis)/(2*sigmac*sigmac) );

//              float norIAndIJ=sqrt( (norx*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x+
//                      nory*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y+
//                      norz*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z)*
//                      (norx*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x+
//                       nory*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y+
//                       norz*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z) );

//              float yy=(norx-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x)*(norx-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x)+
//                      (nory-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y)*(nory-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y)+
//                      (norz-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z)*(norz-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z);
//              float wsigmas=exp( -yy/(2*sigmas*sigmas) );
//              float nijqikij=(cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x*(searchPoint.x- cloud_RadFilted.points[ pointIdxNKNSearch[i] ].x) )+
//                      (cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y*(searchPoint.y- cloud_RadFilted.points[ pointIdxNKNSearch[i] ].y) )+
//                      (cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z*(searchPoint.z- cloud_RadFilted.points[ pointIdxNKNSearch[i] ].z) );
//              float num=wsigmac*dis*wsigmas*norIAndIJ*nijqikij;
//              float den=wsigmac*dis*wsigmas*norIAndIJ;
//              snum+=num;
//              sden+=den;
//          }
//          d=snum/sden;
//        }

//        searchPoint.x+=d*norx;
//        searchPoint.y+=d*nory;
//        searchPoint.z+=d*norz;
//         (*item_cloud).cloud_filted.points.push_back(searchPoint);
//    }

    float min_x2 = 100000;
    float min_y2 = 100000;
    float min_z2 = 100000;
    float max_x2 = -100000;
    float max_y2 = -100000;
    float max_z2 = -100000;

    for(unsigned int i=0; i<(*item_cloud).cloud_filted.points.size(); i++)
    {
        float x = (*item_cloud).cloud_filted.points[i].x;
        float y = (*item_cloud).cloud_filted.points[i].y;
        float z = (*item_cloud).cloud_filted.points[i].z;
        min_x2 = min_x2 < x ? min_x2 : x;
        min_y2 = min_y2 < y ? min_y2 : y;
        min_z2 = min_z2 < z ? min_z2 : z;
        max_x2 = max_x2 > x ? max_x2 : x;
        max_y2 = max_y2 > y ? max_y2 : y;
        max_z2 = max_z2 > z ? max_z2 : z;
    }

    (*item_cloud).min_filted[0] = min_x2;
    (*item_cloud).min_filted[1] = min_y2;
    (*item_cloud).min_filted[2] = min_z2;
    (*item_cloud).max_filted[0] = max_x2;
    (*item_cloud).max_filted[0] = max_y2;
    (*item_cloud).max_filted[0] = max_z2;

    QWidget *temp = new QWidget;
    QMessageBox::information(temp,"完成","双边滤波完成!");
    delete temp;

    emit filter_ok();

}
void FilterCloud::thick_Filter(float max_thick, float min_thick, float selected_thick)
{
    float ave_thick = selected_thick/2;

    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;



    pcl::PointCloud<pcl::PointXYZ> cloud_unfilted;
    cloud_unfilted.points.clear();

    float min_x = 100000;
    float min_y = 100000;
    float min_z = 100000;
    float max_x = -100000;
    float max_y = -100000;
    float max_z = -100000;

    for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
        p.y = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
        p.z = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
        cloud_unfilted.points.push_back(p);
        min_x = min_x < p.x ? min_x : p.x;
        min_y = min_y < p.y ? min_y : p.y;
        min_z = min_z < p.z ? min_z : p.z;
        max_x = max_x > p.x ? max_x : p.x;
        max_y = max_y > p.y ? max_y : p.y;
        max_z = max_z > p.z ? max_z : p.z;
    }


    float WIDTH = 0.3;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (WIDTH);

    octree.defineBoundingBox(min_x-0.05,min_y-0.05,min_z-0.05,max_x+0.05,max_y+0.05,max_z+0.05);
    octree.setInputCloud(cloud_unfilted.makeShared());
    octree.addPointsFromInputCloud();

    pcl::PointXYZ searchPoint;
    searchPoint.x = min_x;
    searchPoint.y = min_y;
    searchPoint.z = min_z;

    for(searchPoint.x = min_x;searchPoint.x<=max_x;searchPoint.x+=WIDTH)
        {
            for(searchPoint.y = min_y;searchPoint.y<=max_y;searchPoint.y+=WIDTH)
            {
                double temp_min_z = 100000;
                double temp_max_z = -100000;

                vector<Point> temp_point_vec;

                for(searchPoint.z = min_z;searchPoint.z<=max_z;searchPoint.z+=WIDTH)
                {
                    vector<int> pointIdxVec;
                    if (octree.voxelSearch (searchPoint, pointIdxVec))
                    {
                        for (size_t i = 0; i < pointIdxVec.size (); ++i)
                        {
                            Point p;
                            p.x = cloud_unfilted.points[pointIdxVec[i]].x;
                            p.y = cloud_unfilted.points[pointIdxVec[i]].y;
                            p.z = cloud_unfilted.points[pointIdxVec[i]].z;
                            temp_point_vec.push_back(p);
                            if(p.z < temp_min_z)
                                temp_min_z = p.z;
                            if(p.z > temp_max_z)
                                temp_max_z = p.z;
                        }
                    }
                }

                if(!temp_point_vec.empty())
                {
                    vector<Point>::iterator item;
                    for(item=temp_point_vec.begin(); item!=temp_point_vec.end(); item++)
                    {
                        //if(item->z < temp_min_z+0.02)
                        double averagez=(temp_max_z+temp_min_z)/2;
                        if( ( (temp_max_z-temp_min_z)>min_thick)&&( (temp_max_z-temp_min_z)<max_thick) )
                        {
                            if( (item->z > (averagez-ave_thick))&&(item->z < (averagez+ave_thick)) )
                            {
                                pcl::PointXYZ point_selected;
                                point_selected.x = item->x;
                                point_selected.y = item->y;
                                point_selected.z = item->z;

                                (*item_cloud).cloud_filted.points.push_back(point_selected);
                            }
                        }
                        else
                        {
                            pcl::PointXYZ point_selected;
                            point_selected.x = item->x;
                            point_selected.y = item->y;
                            point_selected.z = item->z;

                            (*item_cloud).cloud_filted.points.push_back(point_selected);
                        }
                    }
                }
            }
        }

    float min_x2 = 100000;
    float min_y2 = 100000;
    float min_z2 = 100000;
    float max_x2 = -100000;
    float max_y2 = -100000;
    float max_z2 = -100000;

    for(unsigned int i=0; i<(*item_cloud).cloud_filted.points.size(); i++)
    {
        float x = (*item_cloud).cloud_filted.points[i].x;
        float y = (*item_cloud).cloud_filted.points[i].y;
        float z = (*item_cloud).cloud_filted.points[i].z;
        min_x2 = min_x2 < x ? min_x2 : x;
        min_y2 = min_y2 < y ? min_y2 : y;
        min_z2 = min_z2 < z ? min_z2 : z;
        max_x2 = max_x2 > x ? max_x2 : x;
        max_y2 = max_y2 > y ? max_y2 : y;
        max_z2 = max_z2 > z ? max_z2 : z;
    }

    (*item_cloud).min_filted[0] = min_x2;
    (*item_cloud).min_filted[1] = min_y2;
    (*item_cloud).min_filted[2] = min_z2;
    (*item_cloud).max_filted[0] = max_x2;
    (*item_cloud).max_filted[0] = max_y2;
    (*item_cloud).max_filted[0] = max_z2;

    QWidget *temp = new QWidget;
    QMessageBox::information(temp,"完成","厚度滤波完成!");
    delete temp;

    emit filter_ok();
}


void FilterCloud::las2triangulation()
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    int show_index = (*item_cloud).show_index;

    long point_num;

    if(show_index == 0)
    {
        (*item_cloud).cloud_triangles.clear();
        point_num = (*item_cloud).cloud.points.size();
        PointNode *pointc=new PointNode[point_num];
        for(int i=0;i<point_num;i++)
        {
            PointStorage_create_point(pointc+i, (*item_cloud).cloud.points[i].x,
                                                (*item_cloud).cloud.points[i].y,
                                                (*item_cloud).cloud.points[i].z);
        }
        fprintf(stderr, "computing TIN ... \n");
        TINclean(point_num, 0);
        printf("tin, point num  =%d!!!\n", point_num);

        int count = 0;
        for (int i = 0; i < point_num; i++)
        {
            if ((pointc + i)->delete_sn == 0)
            {
                TINadd((pointc + i)->xyz);
                count++;
            }
        }

        printf("tin, count num  =%d!!!\n", count);
        TINfinish();
        cout<< "TIN size is " <<TINget_size()<<endl;

        TINtriangle *output;
        for (int i=0; i<TINget_size();i++)
        {
             output=TINget_triangle(i);
             if(output->V[0])
             {
                 Triangle temp;
                 temp.p[0].x = *(output->V[0]);
                 temp.p[0].y = *(output->V[0]+1);
                 temp.p[0].z = *(output->V[0]+2);

                 temp.p[1].x = *(output->V[1]);
                 temp.p[1].y = *(output->V[1]+1);
                 temp.p[1].z = *(output->V[1]+2);

                 temp.p[2].x = *(output->V[2]);
                 temp.p[2].y = *(output->V[2]+1);
                 temp.p[2].z = *(output->V[2]+2);

                 (*item_cloud).cloud_triangles.push_back(temp);

              }
              else
              {

              }
        }
        delete []pointc;
    }
    else if(show_index==1)
    {
        (*item_cloud).cloud_filted_triangles.clear();
        point_num = (*item_cloud).cloud_filted.points.size();
        PointNode *pointc=new PointNode[point_num];
        for(int i=0;i<point_num;i++)
        {
            PointStorage_create_point(pointc+i, (*item_cloud).cloud_filted.points[i].x,
                                                (*item_cloud).cloud_filted.points[i].y,
                                                (*item_cloud).cloud_filted.points[i].z);
        }
        fprintf(stderr, "computing TIN ... \n");
        TINclean(point_num, 0);
        printf("tin, point num  =%d!!!\n", point_num);

        int count = 0;
        for (int i = 0; i < point_num; i++)
        {
            if ((pointc + i)->delete_sn == 0)
            {
                TINadd((pointc + i)->xyz);
                count++;
            }
        }

        printf("tin, count num  =%d!!!\n", count);
        TINfinish();
        cout<< "TIN size is " <<TINget_size()<<endl;

        TINtriangle *output;
        for (int i=0; i<TINget_size();i++)
        {
             output=TINget_triangle(i);
             if(output->V[0])
             {
                 Triangle temp;
                 temp.p[0].x = *(output->V[0]);
                 temp.p[0].y = *(output->V[0]+1);
                 temp.p[0].z = *(output->V[0]+2);

                 temp.p[1].x = *(output->V[1]);
                 temp.p[1].y = *(output->V[1]+1);
                 temp.p[1].z = *(output->V[1]+2);

                 temp.p[2].x = *(output->V[2]);
                 temp.p[2].y = *(output->V[2]+1);
                 temp.p[2].z = *(output->V[2]+2);

                 (*item_cloud).cloud_filted_triangles.push_back(temp);

              }
              else
              {

              }
        }
        delete []pointc;
    }

    QWidget *temp = new QWidget;
    QMessageBox::information(temp,"完成","三角化完成!");
    delete temp;

   emit triangulate_ok();


}

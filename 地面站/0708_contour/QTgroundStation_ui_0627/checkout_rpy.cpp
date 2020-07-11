#include "checkout_rpy.h"



Checkout_rpy::Checkout_rpy(QObject *parent) : QObject(parent)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);

    cloud1 = temp1;
    cloud2 = temp2;
}

void Checkout_rpy::generateCbn(double roll,double pitch,double yaw)
{
    double cosPitch,sinPitch;
    double cosRoll,sinRoll;
    double cosYaw,sinYaw;

    cosPitch = cos((pitch)*DEG2RAD);
    sinPitch = sin((pitch)*DEG2RAD);

    cosRoll = cos((roll)*DEG2RAD);
    sinRoll = sin((roll)*DEG2RAD);

    cosYaw = cos((yaw)*DEG2RAD);
    sinYaw = sin((yaw)*DEG2RAD);

    Cbn[0][0] = cosYaw*cosPitch;
    Cbn[0][1] = -sinYaw*cosRoll + cosYaw*sinPitch*sinRoll;
    Cbn[0][2] = sinYaw*sinRoll + cosYaw*sinPitch*cosRoll;

    Cbn[1][0] = sinYaw*cosPitch;
    Cbn[1][1] = cosYaw*cosRoll + sinYaw*sinPitch*sinRoll;
    Cbn[1][2] = sinYaw*sinPitch*cosRoll - cosYaw*sinRoll;

    Cbn[2][0] = -sinPitch;
    Cbn[2][1] = cosPitch*sinRoll;
    Cbn[2][2] = cosPitch*cosRoll;

    return;
}


void Checkout_rpy::calcu_lidar_point(vector <Ori_data> &tdata, Trans_para3 para, pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud)
{

    double xl,yl,zl;
    double XL,YL,ZL;
    double x,y,z;
    double delta_xb = 1.05, delta_yb = 0.02, delta_zb = 0.32;

    p_cloud->points.clear();

    for(unsigned int i=0; i<tdata.size(); i++)
    {
        generateCbn(tdata[i].euler[0],tdata[i].euler[1],tdata[i].euler[2]);
        xl = tdata[i].p_laser.x;
        yl = tdata[i].p_laser.y;
        zl = tdata[i].p_laser.z;

        if(((xl<=0.01)&&(xl>=-0.01))||((yl<=0.01)&&(yl>=-0.01))||((zl<=0.01)&&(zl)>=-0.01))
        {
            x = 0;
            y = 0;
            z = 0;
        }
        else
        {
           // cout<<"xyzl: "<<xl<<" "<<yl<<" "<<zl<<endl;
            XL = xl-para.dy*DEG2RAD*yl+para.dp*DEG2RAD*zl+delta_xb;
            YL = para.dy*DEG2RAD*xl+yl-para.dr*DEG2RAD*zl+delta_yb;
            ZL = -para.dp*DEG2RAD*xl+para.dr*DEG2RAD*yl+zl+delta_zb;
          //  cout<<"xyzL: "<<XL<<" "<<YL<<" "<<ZL<<endl;

            x = XL*Cbn[0][0]+YL*Cbn[0][1]+ZL*Cbn[0][2]+tdata[i].posNED[0];
            y = XL*Cbn[1][0]+YL*Cbn[1][1]+ZL*Cbn[1][2]+tdata[i].posNED[1];
            z = XL*Cbn[2][0]+YL*Cbn[2][1]+ZL*Cbn[2][2]+tdata[i].posNED[2];

          //  cout<<"xyz: "<<x<<" "<<y<<" "<<z<<endl;
        }
        pcl::PointXYZ p;
        p.x = x;
        p.y = y;
        p.z = z;

        p_cloud->points.push_back(p);
    }
   // cout<<"calcu_lidar_point ok!"<<endl;
}


double Checkout_rpy::calcu_sum_distance(Trans_para3 para)
{

    calcu_lidar_point(ori_cloud1,para,cloud1);
    calcu_lidar_point(ori_cloud2,para,cloud2);

    cout<<"drpy: "<<para.dr<<" "<<para.dp<<" "<<para.dy<<endl;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud(cloud1);

   // cout<<"kdtree build ok!"<<endl;

    double sum_distance = 0;


    for(unsigned int i=0; i<cloud2->points.size(); i++)
    {
        int k = 1;

        pcl::PointXYZ searchPoint;
        searchPoint.x = cloud2->points[i].x;
        searchPoint.y = cloud2->points[i].y;
        searchPoint.z = cloud2->points[i].z;

       //  cout<<"xyz2: "<<searchPoint.x<<" "<<searchPoint.y<<" "<<searchPoint.z<<endl;

        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        if(kdtree.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {

            for(size_t j = 0; j < pointIdxNKNSearch.size (); j++)
            {
              //  cout<<"xyz1: "<<cloud1->points[pointIdxNKNSearch[j]].x<<" "<<cloud1->points[pointIdxNKNSearch[j]].y<<" "<<cloud1->points[pointIdxNKNSearch[j]].z<<endl;
                sum_distance += pointNKNSquaredDistance[j];
              //  cout<<i<<" distance:"<<pointNKNSquaredDistance[j]<<endl;;
            }
        }

    }
    cout<<"sum_distance:"<<sum_distance<<endl;
    return sum_distance;
}


Trans_para3 Checkout_rpy::One_D_Search(Trans_para3 init_para,double step_length[],double err_allow[],int max_step_num,int max_motified_step_num)
{
    int max_step_num_copy = max_step_num;
    static unsigned long time = 0;

    if(max_motified_step_num == 0)
    {
        return init_para;
    }

    double sum_distance0 = calcu_sum_distance(init_para);

    Trans_para3 para1,para2;
    double sum_distance1,sum_distance2;

    para2.dr = init_para.dr + step_length[0];
    para2.dp = init_para.dp + step_length[1];
    para2.dy = init_para.dy + step_length[2];
    sum_distance2 = calcu_sum_distance(para2);

    if(sum_distance2 >= sum_distance0)
    {
        para1 = para2;
        sum_distance1 = sum_distance2;

        step_length[0] = -step_length[0];
        step_length[1] = -step_length[1];
        step_length[2] = -step_length[2];

        para2.dr = init_para.dr + step_length[0];
        para2.dp = init_para.dp + step_length[1];
        para2.dy = init_para.dy + step_length[2];
        sum_distance2 = calcu_sum_distance(para2);

        while(sum_distance2 <= sum_distance0 && max_step_num_copy > 0)
        {
            para1 = init_para;
            sum_distance1 = sum_distance0;
            init_para = para2;
            sum_distance0 = sum_distance2;

            para2.dr = init_para.dr + step_length[0];
            para2.dp = init_para.dp + step_length[1];
            para2.dy = init_para.dy + step_length[2];
            sum_distance2 = calcu_sum_distance(para2);

            max_step_num_copy--;
            time++;
        }
    }
    else
    {
        para1 = init_para;
        sum_distance1 = sum_distance0;
        init_para = para2;
        sum_distance0 = sum_distance2;

        para2.dr = init_para.dr + step_length[0];
        para2.dp = init_para.dp + step_length[1];
        para2.dy = init_para.dy + step_length[2];
        sum_distance2 = calcu_sum_distance(para2);

        while(sum_distance2 <= sum_distance0 && max_step_num_copy > 0)
        {
            para1 = init_para;
            sum_distance1 = sum_distance0;
            init_para = para2;
            sum_distance0 = sum_distance2;

            para2.dr = init_para.dr + step_length[0];
            para2.dp = init_para.dp + step_length[1];
            para2.dy = init_para.dy + step_length[2];
            sum_distance2 = calcu_sum_distance(para2);

            max_step_num_copy--;
            time++;
        }
    }
    if(fabs(para2.dr-para1.dr)<=err_allow[0] && fabs(para2.dp-para1.dp)<=err_allow[1] && fabs(para2.dy-para1.dy)<=err_allow[2])
    {
        Trans_para3 result;
        result.dr = (para1.dr+para2.dr)/2;
        result.dp = (para1.dp+para2.dp)/2;
        result.dy = (para1.dy+para2.dy)/2;

        return result;
    }
    else
    {
        double step_length_motified[3];
        step_length_motified[0] = step_length[0]/10.0f;
        step_length_motified[1] = step_length[1]/10.0f;
        step_length_motified[2] = step_length[2]/10.0f;


        Trans_para3 para_motified;
        para_motified.dr = (para1.dr+para2.dr)/2;
        para_motified.dp = (para1.dp+para2.dp)/2;
        para_motified.dy = (para1.dy+para2.dy)/2;

        return One_D_Search(para_motified,step_length_motified,err_allow,max_step_num,max_motified_step_num--);

    }


}


Trans_para3 Checkout_rpy::Powell_Search(Trans_para3 init_para,double init_step_length[],double err_allow_whole[],int max_ring_time)
{
    Trans_para3 status = init_para;

    while(max_ring_time--)
    {
        Trans_para3 ring_init_para = status;

        for(int i=0; i<3; i++)
        {
            double step_length[3] = {0,0,0};
            double err_allow[3] = {0,0,0};
            step_length[i] = init_step_length[i];
            err_allow[i] = err_allow_whole[i];
            status = One_D_Search(status,step_length,err_allow,20,2);

           // cout<<"drpy: "<<status.dr<<" "<<status.dp<<" "<<status.dy<<endl;
        }
        double step_length[3] = {0,0,0};
        step_length[0] = (status.dr - ring_init_para.dr)/2.0f;
        step_length[1] = (status.dp - ring_init_para.dp)/2.0f;
        step_length[2] = (status.dy - ring_init_para.dy)/2.0f;

        status = One_D_Search(status,step_length,err_allow_whole,20,2);

        if(fabs(status.dr-ring_init_para.dr)<=err_allow_whole[0] && fabs(status.dp-ring_init_para.dp)<=err_allow_whole[1] && \
                fabs(status.dy-ring_init_para.dy)<=err_allow_whole[2])
            break;
    }
    return status;

}




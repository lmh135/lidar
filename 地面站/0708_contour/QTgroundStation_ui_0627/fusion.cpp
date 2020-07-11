#include "fusion.h"



void Fusion_cloud_pic::generateCbn(double roll,double pitch,double yaw)
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

Fusion_cloud_pic::Fusion_cloud_pic(QObject *parent) : QObject(parent)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp3(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp4(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_pic_selected = temp1;
    cloud_world_selected = temp2;
    cloud_pic_whole = temp3;
    cloud_world_whole = temp4;

}


double Fusion_cloud_pic::calcu_sum_distance(Trans_para7 para)
{
    double x0 = para.P0.x;
    double y0 = para.P0.y;
    double z0 = para.P0.z;  
    double roll = para.roll;
    double pitch = para.pitch;
    double yaw = para.yaw;
    double u = para.u;

    double sum_distance = 0;

    generateCbn(roll,pitch,yaw);

    for(unsigned int i=0; i<cloud_pic_selected->points.size(); i++)
    {
        int k = 1;
        pcl::PointXYZ p = cloud_pic_selected->points[i];
        double x = p.x;
        double y = p.y;
        double z = p.z;
        pcl::PointXYZ searchPoint;
        searchPoint.x = (Cbn[0][0]*(x-x0)+Cbn[0][1]*(y-y0)+Cbn[0][2]*(z-z0))/(1+u);
        searchPoint.y = (Cbn[1][0]*(x-x0)+Cbn[1][1]*(y-y0)+Cbn[1][2]*(z-z0))/(1+u);
        searchPoint.z = (Cbn[2][0]*(x-x0)+Cbn[2][1]*(y-y0)+Cbn[2][2]*(z-z0))/(1+u);

        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        if(kdtree_world.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {

            for(size_t j = 0; j < pointIdxNKNSearch.size (); j++)
            {
                sum_distance += pointNKNSquaredDistance[j];
               // cout<<i<<" distance:"<<pointNKNSquaredDistance[j]<<endl;;
            }
        }
    }
    cout<<"sum_distance:"<<sum_distance<<endl;
    return sum_distance;
}

void Fusion_cloud_pic::init_world_cloud_selected(pcl::PointCloud<pcl::PointXYZ> world_cloud_selected)
{
    cloud_world_selected->points.clear();

    cloud_world_selected = world_cloud_selected.makeShared();

    kdtree_world.setInputCloud (cloud_world_selected);
}

void Fusion_cloud_pic::init_pic_cloud_selected(pcl::PointCloud<pcl::PointXYZ> pic_cloud_selected)
{
    cloud_pic_selected->points.clear();

    cloud_pic_selected = pic_cloud_selected.makeShared();

    kdtree_pic.setInputCloud (cloud_pic_selected);
}

void Fusion_cloud_pic::init_pic_cloud_whole(pcl::PointCloud<pcl::PointXYZ> pic_cloud_whole, std::vector<Color>& color)
{
    cloud_pic_whole->points.clear();

    cloud_pic_whole = pic_cloud_whole.makeShared();

    for(unsigned int i=0; i<color.size(); i++)
    {
        Color rgb = color[i];
        pic_color.push_back(rgb);
    }

}

void Fusion_cloud_pic::init_world_cloud_whole(pcl::PointCloud<pcl::PointXYZ> world_cloud_whole)
{
    cloud_world_whole->points.clear();

    cloud_world_whole = world_cloud_whole.makeShared();
}

//输入:初始变换参数，各参数步长，各参数允许误差，最大步数，最大变步长数
//输出:优化参数，优化最短距离和，计算次数总和
Search_result Fusion_cloud_pic::One_D_Search(Trans_para7 init_para,double step_length[],double err_allow[],int max_step_num,int max_motified_step_num)
{
    int max_step_num_copy = max_step_num;
    static unsigned long time = 0;

    if(max_motified_step_num == 0)
    {
        Search_result result;
        result.para = init_para;
        result.optimized_distance = calcu_sum_distance(init_para);
        result.time = time;
        return result;
    }

    double sum_distance0 = calcu_sum_distance(init_para);

    Trans_para7 para1,para2;
    double sum_distance1,sum_distance2;

    para2.P0.x = init_para.P0.x + step_length[0];
    para2.P0.y = init_para.P0.y + step_length[1];
    para2.P0.z = init_para.P0.z + step_length[2];
    para2.roll = init_para.roll + step_length[3];
    para2.pitch = init_para.pitch + step_length[4];
    para2.yaw = init_para.yaw + step_length[5];
    para2.u = init_para.u + step_length[6];
    sum_distance2 = calcu_sum_distance(para2);

    if(sum_distance2 >= sum_distance0)
    {
        para1 = para2;
        sum_distance1 = sum_distance2;

        step_length[0] = -step_length[0];
        step_length[1] = -step_length[1];
        step_length[2] = -step_length[2];
        step_length[3] = -step_length[3];
        step_length[4] = -step_length[4];
        step_length[5] = -step_length[5];
        step_length[6] = -step_length[6];

        para2.P0.x = init_para.P0.x + step_length[0];
        para2.P0.y = init_para.P0.y + step_length[1];
        para2.P0.z = init_para.P0.z + step_length[2];
        para2.roll = init_para.roll + step_length[3];
        para2.pitch = init_para.pitch + step_length[4];
        para2.yaw = init_para.yaw + step_length[5];
        para2.u = init_para.u + step_length[6];
        sum_distance2 = calcu_sum_distance(para2);

        while(sum_distance2 <= sum_distance0 && max_step_num_copy > 0)
        {
            para1 = init_para;
            sum_distance1 = sum_distance0;
            init_para = para2;
            sum_distance0 = sum_distance2;

            para2.P0.x = init_para.P0.x + step_length[0];
            para2.P0.y = init_para.P0.y + step_length[1];
            para2.P0.z = init_para.P0.z + step_length[2];
            para2.roll = init_para.roll + step_length[3];
            para2.pitch = init_para.pitch + step_length[4];
            para2.yaw = init_para.yaw + step_length[5];
            para2.u = init_para.u + step_length[6];
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

        para2.P0.x = init_para.P0.x + step_length[0];
        para2.P0.y = init_para.P0.y + step_length[1];
        para2.P0.z = init_para.P0.z + step_length[2];
        para2.roll = init_para.roll + step_length[3];
        para2.pitch = init_para.pitch + step_length[4];
        para2.yaw = init_para.yaw + step_length[5];
        para2.u = init_para.u + step_length[6];
        sum_distance2 = calcu_sum_distance(para2);

        while(sum_distance2 <= sum_distance0 && max_step_num_copy > 0)
        {
            para1 = init_para;
            sum_distance1 = sum_distance0;
            init_para = para2;
            sum_distance0 = sum_distance2;

            para2.P0.x = init_para.P0.x + step_length[0];
            para2.P0.y = init_para.P0.y + step_length[1];
            para2.P0.z = init_para.P0.z + step_length[2];
            para2.roll = init_para.roll + step_length[3];
            para2.pitch = init_para.pitch + step_length[4];
            para2.yaw = init_para.yaw + step_length[5];
            para2.u = init_para.u + step_length[6];
            sum_distance2 = calcu_sum_distance(para2);

            max_step_num_copy--;
            time++;
        }
    }
    if(fabs(para2.P0.x-para1.P0.x)<=err_allow[0] && fabs(para2.P0.y-para1.P0.y)<=err_allow[1] && fabs(para2.P0.z-para1.P0.z)<=err_allow[2] && \
            fabs(para2.roll-para1.roll)<=err_allow[3] && fabs(para2.pitch-para1.pitch)<=err_allow[4] && fabs(para2.yaw-para1.yaw)<=err_allow[5] && \
            fabs(para2.u-para1.u)<=err_allow[6] )
    {
        Search_result result;
        result.para.P0.x = (para1.P0.x+para2.P0.x)/2;
        result.para.P0.y = (para1.P0.y+para2.P0.y)/2;
        result.para.P0.z = (para1.P0.z+para2.P0.z)/2;
        result.para.roll = (para1.roll+para2.roll)/2;
        result.para.pitch = (para1.pitch+para2.pitch)/2;
        result.para.yaw = (para1.pitch+para2.yaw)/2;
        result.para.u = (para1.u+para2.u)/2;
        result.optimized_distance = calcu_sum_distance(result.para);
        result.time = time;

        return result;
    }
    else
    {
        double step_length_motified[7];
        step_length_motified[0] = step_length[0]/10.0f;
        step_length_motified[1] = step_length[1]/10.0f;
        step_length_motified[2] = step_length[2]/10.0f;
        step_length_motified[3] = step_length[3]/10.0f;
        step_length_motified[4] = step_length[4]/10.0f;
        step_length_motified[5] = step_length[5]/10.0f;
        step_length_motified[6] = step_length[6]/10.0f;

        Trans_para7 para_motified;
        para_motified.P0.x = (para1.P0.x+para2.P0.x)/2;
        para_motified.P0.y = (para1.P0.y+para2.P0.y)/2;
        para_motified.P0.z = (para1.P0.z+para2.P0.z)/2;
        para_motified.roll = (para1.roll+para2.roll)/2;
        para_motified.pitch = (para1.pitch+para2.pitch)/2;
        para_motified.yaw = (para1.pitch+para2.yaw)/2;
        para_motified.u = (para1.u+para2.u)/2;

        return One_D_Search(para_motified,step_length_motified,err_allow,max_step_num,--max_motified_step_num);

    }

}


//输入:初始变换参数，各参数步长，最大轮次
//输出:优化参数，优化最短距离和
Search_result Fusion_cloud_pic::Powell_Search(Trans_para7 init_para,double init_step_length[],double err_allow_whole[],int max_ring_time)
{
    Search_result status;


    status.para = init_para;

    while(max_ring_time--)
    {
        Trans_para7 ring_init_para = status.para;

        for(int i=0; i<7; i++)
        {
            double step_length[7] = {0,0,0,0,0,0,0};
            double err_allow[7] = {0,0,0,0,0,0,0};
            step_length[i] = init_step_length[i];
            err_allow[i] = err_allow_whole[i];
            status = One_D_Search(status.para,step_length,err_allow,10,2);
        }
        double step_length[7] = {0,0,0,0,0,0,0};
        step_length[0] = (status.para.P0.x - ring_init_para.P0.x)/2.0f;
        step_length[1] = (status.para.P0.y - ring_init_para.P0.y)/2.0f;
        step_length[2] = (status.para.P0.z - ring_init_para.P0.z)/2.0f;
        step_length[3] = (status.para.roll - ring_init_para.roll)/2.0f;
        step_length[4] = (status.para.pitch - ring_init_para.pitch)/2.0f;
        step_length[5] = (status.para.yaw - ring_init_para.yaw)/2.0f;
        step_length[6] = (status.para.u - ring_init_para.u)/2.0f;

        status = One_D_Search(status.para,step_length,err_allow_whole,10,2);

        if(fabs(status.para.P0.x-ring_init_para.P0.x)<=err_allow_whole[0] && fabs(status.para.P0.y-ring_init_para.P0.y)<=err_allow_whole[1] && \
                fabs(status.para.P0.z-ring_init_para.P0.z)<=err_allow_whole[2] && fabs(status.para.roll-ring_init_para.roll)<=err_allow_whole[3] && \
                fabs(status.para.pitch-ring_init_para.pitch)<=err_allow_whole[4] && fabs(status.para.yaw-ring_init_para.yaw)<=err_allow_whole[5] && \
                fabs(status.para.u-ring_init_para.u)<=err_allow_whole[6])
            break;
    }
    return status;



}



void Fusion_cloud_pic::Find_color(Trans_para7 para)
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    (*item_cloud).color.clear();

    double x0 = para.P0.x;
    double y0 = para.P0.y;
    double z0 = para.P0.z;
    double roll = para.roll;
    double pitch = para.pitch;
    double yaw = para.yaw;
    double u = para.u;

    generateCbn(roll,pitch,yaw);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_pic_whole;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pic_whole_trans(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_pic_whole_trans->points.clear();

    for(unsigned int i=0; i<cloud_pic_whole->points.size(); i++)
    {
        pcl::PointXYZ p = cloud_pic_whole->points[i];
        double x = p.x;
        double y = p.y;
        double z = p.z;
        pcl::PointXYZ point_trans;
        point_trans.x = (Cbn[0][0]*(x-x0)+Cbn[0][1]*(y-y0)+Cbn[0][2]*(z-z0))/(1+u);
        point_trans.y = (Cbn[1][0]*(x-x0)+Cbn[1][1]*(y-y0)+Cbn[1][2]*(z-z0))/(1+u);
        point_trans.z = (Cbn[2][0]*(x-x0)+Cbn[2][1]*(y-y0)+Cbn[2][2]*(z-z0))/(1+u);

        cloud_pic_whole_trans->points.push_back(point_trans);
    }
    kdtree_pic_whole.setInputCloud(cloud_pic_whole_trans);



    for(unsigned int i=0; i<cloud_world_whole->points.size(); i++)
    {
        int k = 1;
        pcl::PointXYZ searchPoint = cloud_world_whole->points[i];

        (*item_cloud).cloud.points.push_back(searchPoint);

        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);

        if(kdtree_pic_whole.nearestKSearch(searchPoint, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
           // if(pointNKNSquaredDistance[0]<2.0f)
            {
                Color rgb = pic_color[pointIdxNKNSearch[0]];
                (*item_cloud).color.push_back(rgb);
            }
          //  else
          //  {
          //      Color rgb;
          //      rgb.r = 0.0f;
          //      rgb.g = 0.0f;
          //      rgb.b = 0.0f;
          //      (*item_cloud).color.push_back(rgb);
          //  }
        }
        else
        {
            cout<<"false"<<endl;
            Color rgb;
            rgb.r = 0.0f;
            rgb.g = 0.0f;
            rgb.b = 0.0f;
            (*item_cloud).color.push_back(rgb);
        }
    }
    (*item_cloud).isColor = true;



    float min_x = 10000;
    float min_y = 10000;
    float min_z = 10000;
    float max_x = -10000;
    float max_y = -10000;
    float max_z = -10000;
    for(unsigned int i=0; i<(*item_cloud).cloud.points.size(); i++)
    {
         pcl::PointXYZ p = (*item_cloud).cloud.points[i];
         if(min_x > p.x) min_x = p.x;
         if(min_y > p.y) min_y = p.y;
         if(min_z > p.z) min_z = p.z;
         if(max_x < p.x) max_x = p.x;
         if(max_y < p.y) max_y = p.y;
         if(max_z < p.z) max_z = p.z;
    }
    (*item_cloud).min[0] = min_x;
    (*item_cloud).min[1] = min_y;
    (*item_cloud).min[2] = min_z;
    (*item_cloud).max[0] = max_x;
    (*item_cloud).max[1] = max_y;
    (*item_cloud).max[2] = max_z;



    cout<<"ok~"<<endl;

    return;

}


Trans_para7 Fusion_cloud_pic::calcu_init_para(Trans_para7 init_para, int num)
{
    if(num == 0)
    {
        cout<<"run time out!!"<<endl;
        return init_para;
    }
    double a = init_para.roll*DEG2RAD;
    double b = init_para.pitch*DEG2RAD;
    double y = init_para.yaw*DEG2RAD;
    double u = init_para.u;
    double x0 = init_para.P0.x;
    double y0 = init_para.P0.y;
    double z0 = init_para.P0.z;

    mat Ra0(3,3), Rb0(3,3), Ry0(3,3), dRa_da(3,3), dRb_db(3,3), dRy_dy(3,3);
    mat pos_core(3,1), mid_point1(3,1), mid_point2(3,1), mid_point3(3,1), world_point1(3,1), world_point2(3,1),world_point3(3,1), pic_point1(3,1), pic_point2(3,1), pic_point3(3,1);

    world_point1<<fusion_para.world_point[0].x<<endr<<fusion_para.world_point[0].y<<endr<<fusion_para.world_point[0].z<<endr;
    world_point2<<fusion_para.world_point[1].x<<endr<<fusion_para.world_point[1].y<<endr<<fusion_para.world_point[1].z<<endr;
    world_point3<<fusion_para.world_point[2].x<<endr<<fusion_para.world_point[2].y<<endr<<fusion_para.world_point[2].z<<endr;

//    world_point1<<-23.01<<endr<<-36.91<<endr<<-1.12<<endr;
//    world_point2<<-57.32<<endr<<-34.81<<endr<<2.41<<endr;
//    world_point3<<-67.61<<endr<<-67.19<<endr<<-1.19<<endr;


    pic_point1<<fusion_para.pic_point[0].x<<endr<<fusion_para.pic_point[0].y<<endr<<fusion_para.pic_point[0].z<<endr;
    pic_point2<<fusion_para.pic_point[1].x<<endr<<fusion_para.pic_point[1].y<<endr<<fusion_para.pic_point[1].z<<endr;
    pic_point3<<fusion_para.pic_point[2].x<<endr<<fusion_para.pic_point[2].y<<endr<<fusion_para.pic_point[2].z<<endr;


//    pic_point1<<-20.364<<endr<<-11.519<<endr<<86.234<<endr;
//    pic_point2<<-40.218<<endr<<4.133<<endr<<71.284<<endr;
//    pic_point3<<-59.812<<endr<<-15.764<<endr<<63.311<<endr;


    pos_core<<x0<<endr<<y0<<endr<<z0<<endr;

    Ra0<<1<<0<<0<<endr
      <<0<<cos(a)<<sin(a)<<endr
     <<0<<-sin(a)<<cos(a)<<endr;

    Rb0<<cos(b)<<0<<-sin(b)<<endr
      <<0<<1<<0<<endr
     <<sin(b)<<0<<cos(b)<<endr;

    Ry0<<cos(y)<<sin(y)<<0<<endr
      <<-sin(y)<<cos(y)<<0<<endr
     <<0<<0<<1<<endr;

    dRa_da<<0<<0<<0<<endr
      <<0<<-sin(a)<<cos(a)<<endr
     <<0<<-cos(a)<<-sin(a)<<endr;

    dRb_db<<-sin(b)<<0<<-cos(b)<<endr
      <<0<<0<<0<<endr
     <<cos(b)<<0<<-sin(b)<<endr;

    dRy_dy<<-sin(y)<<cos(y)<<0<<endr
      <<-cos(y)<<-sin(y)<<0<<endr
     <<0<<0<<0<<endr;

    mid_point1 = pos_core+(1+u)*Ra0*Rb0*Ry0*world_point1;
    mid_point2 = pos_core+(1+u)*Ra0*Rb0*Ry0*world_point2;
    mid_point3 = pos_core+(1+u)*Ra0*Rb0*Ry0*world_point3;

    mat k1 = eye(3,3), k2 = eye(3,3), k3 = eye(3,3);

    k1.insert_cols(3,Ra0*Rb0*Ry0*world_point1);
    k1.insert_cols(4,(1+u)*dRa_da*Rb0*Ry0*world_point1);
    k1.insert_cols(5,(1+u)*Ra0*dRb_db*Ry0*world_point1);
    k1.insert_cols(6,(1+u)*Ra0*Rb0*dRy_dy*world_point1);

    k2.insert_cols(3,Ra0*Rb0*Ry0*world_point2);
    k2.insert_cols(4,(1+u)*dRa_da*Rb0*Ry0*world_point2);
    k2.insert_cols(5,(1+u)*Ra0*dRb_db*Ry0*world_point2);
    k2.insert_cols(6,(1+u)*Ra0*Rb0*dRy_dy*world_point2);

    k3.insert_cols(3,Ra0*Rb0*Ry0*world_point3);
    k3.insert_cols(4,(1+u)*dRa_da*Rb0*Ry0*world_point3);
    k3.insert_cols(5,(1+u)*Ra0*dRb_db*Ry0*world_point3);
    k3.insert_cols(6,(1+u)*Ra0*Rb0*dRy_dy*world_point3);

    mat A = k1;
    A.insert_rows(3,k2);
    A.insert_rows(6,k3);

    mat l = pic_point1-mid_point1;
    l.insert_rows(3,pic_point2-mid_point2);
    l.insert_rows(6,pic_point3-mid_point3);

    mat X = solve(A,l);

    Trans_para7 para;
    para.P0.x = x0+X(0,0);
    para.P0.y = y0+X(1,0);
    para.P0.z = z0+X(2,0);
    para.u = u+X(3,0);
    para.roll = (a+X(4,0))/DEG2RAD;
    para.pitch = (b+X(5,0))/DEG2RAD;
    para.yaw = (y+X(6,0))/DEG2RAD;

    if(fabs(X(0,0))<0.0001 && fabs(X(1,0))<0.0001 && fabs(X(2,0))<0.0001 && fabs(X(3,0))<0.0001 && fabs(X(4,0))<0.0001 && fabs(X(5,0))<0.0001 && fabs(X(6,0))<0.0001)
        return para;
    else
        return calcu_init_para(para,--num);

}








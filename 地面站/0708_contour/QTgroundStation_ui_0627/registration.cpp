#include "registration.h"

Registration::Registration(QObject *parent) : QObject(parent)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp3(new pcl::PointCloud<pcl::PointXYZ>);

    cloud1 = temp1;
    cloud2 = temp2;
    cloud_icp = temp3;
}


void Registration::start_registration()
{
    if(registration_para.registration_index==0)
    {
        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"错误","请选择拼接算法!");
        delete temp;
        return;
    }


    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);


    for(vector<MYCloud>::iterator item_temp = cloud_seq.begin(); item_temp!=cloud_seq.end(); item_temp++)
    {
        if(registration_para.cloud1_name == (*item_temp).mycloud_name)
        {
            if((*item_temp).cloud.points.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","点云1为空!");
                delete temp;
                return;
            }
            if((*item_temp).selected_index.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","点云1还未选择公共区域!");
                delete temp;
                return;
            }

            (*item_cloud).cloud_registrated1.points.clear();
            cloud1->points.clear();

            for(unsigned int i=0; i<(*item_temp).cloud.points.size(); i++)
            {
                pcl::PointXYZ p;
                p = (*item_temp).cloud.points[i];
                cloud1->points.push_back(p);
            }
            for(unsigned int i=0; i<(*item_temp).selected_index.size(); i++)
            {
                pcl::PointXYZ p;
                p = (*item_temp).cloud.points[(*item_temp).selected_index[i]];
                temp_cloud1->points.push_back(p);
                (*item_cloud).cloud_registrated1.points.push_back(p);
            }

        }

        if(registration_para.cloud2_name == (*item_temp).mycloud_name)
        {
            if((*item_temp).cloud.points.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","点云2为空!");
                delete temp;
                return;
            }
            if((*item_temp).selected_index.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","点云2还未选择公共区域!");
                delete temp;
                return;
            }

            (*item_cloud).cloud_registrated2.points.clear();
            cloud2->points.clear();

            for(unsigned int i=0; i<(*item_temp).cloud.points.size(); i++)
            {
                pcl::PointXYZ p;
                p = (*item_temp).cloud.points[i];
                cloud2->points.push_back(p);
            }
            for(unsigned int i=0; i<(*item_temp).selected_index.size(); i++)
            {
                pcl::PointXYZ p;
                p = (*item_temp).cloud.points[(*item_temp).selected_index[i]];
                temp_cloud2->points.push_back(p);
                (*item_cloud).cloud_registrated2.points.push_back(p);
            }

        }

    }

    int index = registration_para.registration_index;
    switch(index)
    {
    case 1:
        classical_icp(temp_cloud1,temp_cloud2);
        break;
    case 2:
        MY_icp_search(temp_cloud1,temp_cloud2);
        break;
    default:
        break;

    }


}


void Registration::classical_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud)
{
    if(pick1_cloud->points.empty() || pick2_cloud->points.empty())
        return;

    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    cloud_icp->points.clear();

    *cloud_icp = *pick2_cloud;

    icp.setMaximumIterations (10);

    icp.setInputSource (cloud_icp);   //设置输入的点云
    icp.setInputTarget (pick1_cloud);    //目标点云
    icp.align (*cloud_icp);          //匹配后源点云

    if (icp.hasConverged ())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
    {
        cout << "\nICP has converged, score is " << icp.getFitnessScore () << endl;
        //cout << "\nICP transformation " << iterations << " : cloud_icp -> pick1_cloud" << endl;
        registration_para.transformation_matrix = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (registration_para.transformation_matrix);

        (*item_cloud).cloud_registrated2.points.clear();
        for(unsigned int i=0; i<cloud_icp->points.size(); i++)
        {
            pcl::PointXYZ p;
            p = cloud_icp->points[i];
            (*item_cloud).cloud_registrated2.points.push_back(p);
        }

        (*item_cloud).cloud.points.clear();

        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud2,*trans_cloud,registration_para.transformation_matrix);
        for(unsigned int i=0; i<cloud1->points.size(); i++)
        {
            pcl::PointXYZ p;
            p = cloud1->points[i];
            (*item_cloud).cloud.push_back(p);
        }
        for(unsigned int i=0; i<trans_cloud->points.size(); i++)
        {
            pcl::PointXYZ p;
            p = trans_cloud->points[i];
            (*item_cloud).cloud.push_back(p);
        }

        float min_x = 100000;
        float min_y = 100000;
        float min_z = 100000;
        float max_x = -100000;
        float max_y = -100000;
        float max_z = -100000;

        for(unsigned int i=0; i<(*item_cloud).cloud.points.size(); i++)
        {
            float x = (*item_cloud).cloud.points[i].x;
            float y = (*item_cloud).cloud.points[i].y;
            float z = (*item_cloud).cloud.points[i].z;
            min_x = min_x < x ? min_x : x;
            min_y = min_y < y ? min_y : y;
            min_z = min_z < z ? min_z : z;
            max_x = max_x > x ? max_x : x;
            max_y = max_y > y ? max_y : y;
            max_z = max_z > z ? max_z : z;
        }

        (*item_cloud).min[0] = min_x;
        (*item_cloud).min[1] = min_y;
        (*item_cloud).min[2] = min_z;
        (*item_cloud).max[0] = max_x;
        (*item_cloud).max[1] = max_y;
        (*item_cloud).max[2] = max_z;

        (*item_cloud).show_index = 4;

        emit registration_ok();

        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"完成","经典icp拼接完成!");
        delete temp;
        return;


    }
    else
    {
        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"错误","经典icp拼接失败!");
        delete temp;
        return;
    }

    icp.setMaximumIterations (1);  // 设置为1以便下次调用


}

void Registration::continue_registration()
{
    if(cloud_icp->points.empty())
        return;
    icp.align(*cloud_icp);

    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if (icp.hasConverged ())
    {
        cout << "\nICP has converged, score is " << icp.getFitnessScore () << endl;
        //cout << "\nICP transformation " << iterations << " : cloud_icp -> pick1_cloud" << endl;
        registration_para.transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate!
        print4x4Matrix (registration_para.transformation_matrix);

        (*item_cloud).cloud_registrated2.points.clear();
        for(unsigned int i=0; i<cloud_icp->points.size(); i++)
        {
            pcl::PointXYZ p;
            p = cloud_icp->points[i];
            (*item_cloud).cloud_registrated2.points.push_back(p);
        }

        (*item_cloud).cloud.points.clear();

        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud2,*trans_cloud,registration_para.transformation_matrix);
        for(unsigned int i=0; i<cloud1->points.size(); i++)
        {
            pcl::PointXYZ p;
            p = cloud1->points[i];
            (*item_cloud).cloud.push_back(p);
        }
        for(unsigned int i=0; i<trans_cloud->points.size(); i++)
        {
            pcl::PointXYZ p;
            p = trans_cloud->points[i];
            (*item_cloud).cloud.push_back(p);
        }

        float min_x = 100000;
        float min_y = 100000;
        float min_z = 100000;
        float max_x = -100000;
        float max_y = -100000;
        float max_z = -100000;

        for(unsigned int i=0; i<(*item_cloud).cloud.points.size(); i++)
        {
            float x = (*item_cloud).cloud.points[i].x;
            float y = (*item_cloud).cloud.points[i].y;
            float z = (*item_cloud).cloud.points[i].z;
            min_x = min_x < x ? min_x : x;
            min_y = min_y < y ? min_y : y;
            min_z = min_z < z ? min_z : z;
            max_x = max_x > x ? max_x : x;
            max_y = max_y > y ? max_y : y;
            max_z = max_z > z ? max_z : z;
        }

        (*item_cloud).min[0] = min_x;
        (*item_cloud).min[1] = min_y;
        (*item_cloud).min[2] = min_z;
        (*item_cloud).max[0] = max_x;
        (*item_cloud).max[1] = max_y;
        (*item_cloud).max[2] = max_z;



        emit continue_step_ok();

        return;


    }
    else
    {
        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"错误","经典icp拼接失败!");
        delete temp;
        return;
    }

}

double Registration::calcu_sum_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud,Trans_para6 para)
{
    if(pick1_cloud->points.size()==0 || pick2_cloud->points.size()==0)
        return 0;

    generateCbn(para.roll,para.pitch,para.yaw);


    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud(pick1_cloud);

    double sum_distance = 0;
    for(unsigned int i=0; i<pick2_cloud->points.size(); i++)
    {
        int k = 1;

        pcl::PointXYZ searchPoint;
        double x,y,z;

        x = pick2_cloud->points[i].x;
        y = pick2_cloud->points[i].y;
        z = pick2_cloud->points[i].z;

        searchPoint.x = Cbn[0][0]*x+Cbn[0][1]*y+Cbn[0][2]*z+para.P0.x;
        searchPoint.y = Cbn[1][0]*x+Cbn[1][1]*y+Cbn[1][2]*z+para.P0.y;
        searchPoint.z = Cbn[2][0]*x+Cbn[2][1]*y+Cbn[2][2]*z+para.P0.z;

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

Trans_para6 Registration::One_D_Search(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud,\
                         Trans_para6 init_para,double step_length[],double err_allow[],int max_step_num,int max_motified_step_num)
{
    int max_step_num_copy = max_step_num;
    static unsigned long time = 0;

    if(max_motified_step_num == 0)
    {
        Trans_para6 result = init_para;
        return result;
    }

    double sum_distance0 = calcu_sum_distance(pick1_cloud,pick2_cloud,init_para);

    Trans_para6 para1,para2;
    double sum_distance1,sum_distance2;

    para2.P0.x = init_para.P0.x + step_length[0];
    para2.P0.y = init_para.P0.y + step_length[1];
    para2.P0.z = init_para.P0.z + step_length[2];
    para2.roll = init_para.roll + step_length[3];
    para2.pitch = init_para.pitch + step_length[4];
    para2.yaw = init_para.yaw + step_length[5];

    sum_distance2 = calcu_sum_distance(pick1_cloud,pick2_cloud,para2);

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

        para2.P0.x = init_para.P0.x + step_length[0];
        para2.P0.y = init_para.P0.y + step_length[1];
        para2.P0.z = init_para.P0.z + step_length[2];
        para2.roll = init_para.roll + step_length[3];
        para2.pitch = init_para.pitch + step_length[4];
        para2.yaw = init_para.yaw + step_length[5];
        sum_distance2 = calcu_sum_distance(pick1_cloud,pick2_cloud,para2);

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
            sum_distance2 = calcu_sum_distance(pick1_cloud,pick2_cloud,para2);

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
        sum_distance2 = calcu_sum_distance(pick1_cloud,pick2_cloud,para2);

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
            sum_distance2 = calcu_sum_distance(pick1_cloud,pick2_cloud,para2);

            max_step_num_copy--;
            time++;
        }
    }
    if(fabs(para2.P0.x-para1.P0.x)<=err_allow[0] && fabs(para2.P0.y-para1.P0.y)<=err_allow[1] && fabs(para2.P0.z-para1.P0.z)<=err_allow[2] && \
            fabs(para2.roll-para1.roll)<=err_allow[3] && fabs(para2.pitch-para1.pitch)<=err_allow[4] && fabs(para2.yaw-para1.yaw)<=err_allow[5])
    {
        Trans_para6 result;
        result.P0.x = (para1.P0.x+para2.P0.x)/2;
        result.P0.y = (para1.P0.y+para2.P0.y)/2;
        result.P0.z = (para1.P0.z+para2.P0.z)/2;
        result.roll = (para1.roll+para2.roll)/2;
        result.pitch = (para1.pitch+para2.pitch)/2;
        result.yaw = (para1.pitch+para2.yaw)/2;

        return result;
    }
    else
    {
        double step_length_motified[6];
        step_length_motified[0] = step_length[0]/10.0f;
        step_length_motified[1] = step_length[1]/10.0f;
        step_length_motified[2] = step_length[2]/10.0f;
        step_length_motified[3] = step_length[3]/10.0f;
        step_length_motified[4] = step_length[4]/10.0f;
        step_length_motified[5] = step_length[5]/10.0f;

        Trans_para6 para_motified;
        para_motified.P0.x = (para1.P0.x+para2.P0.x)/2;
        para_motified.P0.y = (para1.P0.y+para2.P0.y)/2;
        para_motified.P0.z = (para1.P0.z+para2.P0.z)/2;
        para_motified.roll = (para1.roll+para2.roll)/2;
        para_motified.pitch = (para1.pitch+para2.pitch)/2;
        para_motified.yaw = (para1.pitch+para2.yaw)/2;

        return One_D_Search(pick1_cloud,pick2_cloud,para_motified,step_length_motified,err_allow,max_step_num,max_motified_step_num--);

    }



}

Trans_para6 Registration::Powell_Search(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud,\
                          Trans_para6 init_para,double init_step_length[],double err_allow_whole[],int max_ring_time)
{
    Trans_para6 status = init_para;

    while(max_ring_time--)
    {
        Trans_para6 ring_init_para = status;

        for(int i=0; i<6; i++)
        {
            double step_length[6] = {0,0,0,0,0,0};
            double err_allow[6] = {0,0,0,0,0,0};
            step_length[i] = init_step_length[i];
            err_allow[i] = err_allow_whole[i];
            status = One_D_Search(pick1_cloud,pick2_cloud,status,step_length,err_allow,20,2);


        }
        double step_length[6] = {0,0,0,0,0,0};
        step_length[0] = (status.P0.x - ring_init_para.P0.x)/2.0f;
        step_length[1] = (status.P0.y - ring_init_para.P0.y)/2.0f;
        step_length[2] = (status.P0.z - ring_init_para.P0.z)/2.0f;
        step_length[3] = (status.roll - ring_init_para.roll)/2.0f;
        step_length[4] = (status.pitch - ring_init_para.pitch)/2.0f;
        step_length[5] = (status.yaw - ring_init_para.yaw)/2.0f;

        status = One_D_Search(pick1_cloud,pick2_cloud,status,step_length,err_allow_whole,20,2);

        if(fabs(status.P0.x-ring_init_para.P0.x)<=err_allow_whole[0] && fabs(status.P0.y-ring_init_para.P0.y)<=err_allow_whole[1] && \
                fabs(status.P0.z-ring_init_para.P0.z)<=err_allow_whole[2] && fabs(status.roll-ring_init_para.roll)<=err_allow_whole[3] && \
                fabs(status.pitch-ring_init_para.pitch)<=err_allow_whole[4] && fabs(status.yaw-ring_init_para.yaw)<=err_allow_whole[5])
            break;
    }
    return status;

}

void Registration::MY_icp_search(pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud)
{

    if(pick1_cloud->points.empty() || pick2_cloud->points.empty())
        return;

    double step_length[6] = {1.0,1.0,1.0,1.0,1.0,1.0};
    double err_allow_whole[6] = {0.03,0.03,0.03,0.03,0.03,0.03};
    Trans_para6 result;
    result = Powell_Search(pick1_cloud,pick2_cloud,registration_para.para6,step_length,err_allow_whole,2);

    cout<<"optimized_xyz:"<<result.P0.x<<" "<<result.P0.y<<" "<<result.P0.z<<endl;
    cout<<"optimized_rpy:"<<result.roll<<" "<<result.pitch<<" "<<result.yaw<<endl;


    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    generateCbn(result.roll,result.pitch,result.yaw);

    (*item_cloud).cloud_registrated2.points.clear();
    for(unsigned int i=0; i<pick2_cloud->points.size(); i++)
    {
        pcl::PointXYZ p;
        double x,y,z;

        x = pick2_cloud->points[i].x;
        y = pick2_cloud->points[i].y;
        z = pick2_cloud->points[i].z;

        p.x = Cbn[0][0]*x+Cbn[0][1]*y+Cbn[0][2]*z+result.P0.x;
        p.y = Cbn[1][0]*x+Cbn[1][1]*y+Cbn[1][2]*z+result.P0.y;
        p.z = Cbn[2][0]*x+Cbn[2][1]*y+Cbn[2][2]*z+result.P0.z;

        (*item_cloud).cloud_registrated2.points.push_back(p);
    }

    (*item_cloud).cloud.points.clear();
    for(unsigned int i=0; i<cloud1->points.size(); i++)
    {
        pcl::PointXYZ p;
        p = cloud1->points[i];
        (*item_cloud).cloud.push_back(p);
    }
    for(unsigned int i=0; i<cloud2->points.size(); i++)
    {
        pcl::PointXYZ p;
        double x,y,z;

        x = cloud2->points[i].x;
        y = cloud2->points[i].y;
        z = cloud2->points[i].z;

        p.x = Cbn[0][0]*x+Cbn[0][1]*y+Cbn[0][2]*z+result.P0.x;
        p.y = Cbn[1][0]*x+Cbn[1][1]*y+Cbn[1][2]*z+result.P0.y;
        p.z = Cbn[2][0]*x+Cbn[2][1]*y+Cbn[2][2]*z+result.P0.z;

        (*item_cloud).cloud.push_back(p);
    }


    float min_x = 100000;
    float min_y = 100000;
    float min_z = 100000;
    float max_x = -100000;
    float max_y = -100000;
    float max_z = -100000;

    for(unsigned int i=0; i<(*item_cloud).cloud.points.size(); i++)
    {
        float x = (*item_cloud).cloud.points[i].x;
        float y = (*item_cloud).cloud.points[i].y;
        float z = (*item_cloud).cloud.points[i].z;
        min_x = min_x < x ? min_x : x;
        min_y = min_y < y ? min_y : y;
        min_z = min_z < z ? min_z : z;
        max_x = max_x > x ? max_x : x;
        max_y = max_y > y ? max_y : y;
        max_z = max_z > z ? max_z : z;
    }

    (*item_cloud).min[0] = min_x;
    (*item_cloud).min[1] = min_y;
    (*item_cloud).min[2] = min_z;
    (*item_cloud).max[0] = max_x;
    (*item_cloud).max[1] = max_y;
    (*item_cloud).max[2] = max_z;

    (*item_cloud).show_index = 4;

    emit registration_ok();

    QWidget *temp = new QWidget;
    QMessageBox::information(temp,"完成","我的icp拼接完成!");
    delete temp;
    return;

}

void Registration::generateCbn(double roll,double pitch,double yaw)
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

void Registration::print4x4Matrix (const Eigen::Matrix4d & matrix)    //打印旋转矩阵和平移矩阵
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

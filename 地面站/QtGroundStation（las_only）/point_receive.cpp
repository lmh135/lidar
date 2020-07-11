#include "point_receive.h"

int num = 0;
bool exec_flag = 1;

pcl::PointCloud<pcl::PointXYZ> points_recv;
pcl::PointCloud<pcl::PointXYZ> points_recv_temp;

Q_point_recv_node::Q_point_recv_node()
{




}

Q_point_recv_node::~Q_point_recv_node()
{


    wait();


}


bool Q_point_recv_node::init()
{


    start();

    return true;

}

void Q_point_recv_node::run()
{
    int val_data,val_place,shmid;
    Points *p_shared;

    Points point_cloud;

    if((val_data = semget(ftok("val_data",0),1,IPC_CREAT|0660)) == -1)
    {
        printf("semget val_data failed.");
        exit(1);
    }

    if((val_place = semget(ftok("val_place",0),1,IPC_CREAT|0660)) == -1)
    {
        printf("semget val_place failed.");
        exit(1);
    }

    if((shmid = shmget((key_t)1023457,sizeof(Points),IPC_CREAT|0660)) == -1)//注意，共享数据结构每改动一次，共享内存名也要改动一次
    {
        printf("shmget point_data failed.");
        exit(1);
    }

    p_shared=(Points*)shmat(shmid,0,0);
    if(p_shared==(Points*)-1)
    {
        printf("shm shmat point_data failed");
        exit(1);
    }

    bool point_new_flag = 0;

    while(exec_flag)
    {
        while((p_shared->flag != 1)&&(exec_flag));
        if(p_shared->flag == 1)
        {
            semaphore_P(val_data);
            num++;
            point_cloud = *p_shared;
            p_shared->flag = 0;
            semaphore_V(val_place);
            point_new_flag = 1;
        }

        if(point_new_flag)
        {
            for(unsigned int i=0; i<share_num; i++)
                for(unsigned int j=0; j<192; j++)
                {
                    pcl::PointXYZ p;
                    p.x = point_cloud.p[i][j].x;
                    p.y = point_cloud.p[i][j].y;
                    p.z = point_cloud.p[i][j].z;
                    points_recv.points.push_back(p);
                    points_recv_temp.points.push_back(p);
                }
            if(points_recv_temp.points.size()>100000)
            {
               points_recv_temp.points.erase(points_recv_temp.points.begin(),(points_recv_temp.points.begin()+=9600));
               min_x = 10000;
               min_y = 10000;
               min_z = 10000;
               max_x = -10000;
               max_y = -10000;
               max_z = -10000;

            }

            for(unsigned int i=0; i<points_recv_temp.points.size(); i++)
            {
                 pcl::PointXYZ p = points_recv_temp.points[i];
                 if(min_x > p.x) min_x = p.x;
                 if(min_y > p.y) min_y = p.y;
                 if(min_z > p.z) min_z = p.z;
                 if(max_x < p.x) max_x = p.x;
                 if(max_y < p.y) max_y = p.y;
                 if(max_z < p.z) max_z = p.z;
            }


            point_new_flag = 0;
            emit update_Point();
            cout<<"num:"<<point_cloud.id_num<<endl;
        }
        //if(!(num%50))
          //  emit update_Point();


    }

}








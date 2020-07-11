#include "registration.h"
#include "ui_registration.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pick1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pick2_cloud(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp(new pcl::PointCloud<pcl::PointXYZ>);
pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
int iterations = 10;



int switch_cloud1_flag = 0;
int switch_cloud2_flag = 0;
int switch_cloud3_flag = 0;

double vorxba1 = 0,voryba1 = 0,vorzba1 = 0;
float max_x1 = -10000,min_x1 = 10000,max_y1 = -10000,min_y1 = 10000,max_z1 = -10000,min_z1 = 10000;

double vorxba2 = 0,voryba2 = 0,vorzba2 = 0;
float max_x2 = -10000,min_x2 = 10000,max_y2 = -10000,min_y2 = 10000,max_z2 = -10000,min_z2 = 10000;

double vorxba3 = 0,voryba3 = 0,vorzba3 = 0;
float max_x3 = -10000,min_x3 = 10000,max_y3 = -10000,min_y3 = 10000,max_z3 = -10000,min_z3 = 10000;

vector<int> selected_index1;
vector<int> selected_index2;


int cloud_selected = 1;

registration::registration(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::registration)
{
    ui->setupUi(this);

    ui->lineEdit1->setStyleSheet("border-width:0;border-style:outset");//设置边框透明
    ui->lineEdit2->setStyleSheet("border-width:0;border-style:outset");
    ui->selected_mode_Edit->setStyleSheet("border-width:0;border-style:outset");

    connect(ui->widget1, SIGNAL(release_left_mouse()), this, SLOT(SelectPoint()));
   // connect(ui->widget2, SIGNAL(release_left_mouse()), this, SLOT(SelectPoint2()));
}

registration::~registration()
{
    delete ui;
}


void registration::SelectPoint()
{
    float press_x,press_y,release_x,release_y;
    press_x = (float)ui->widget1->pressPos.x();
    press_y = (float)ui->widget1->pressPos.y();
    release_x = (float)ui->widget1->releasePos.x();
    release_y = (float)ui->widget1->releasePos.y();

    if(cloud_selected == 1)
    {
        if(cloud1->points.empty())
            return;

        Selection_init(cloud1);
        int hits = DoOpenGLSelection(press_x,press_y,release_x,release_y,cloud1,vorxba1,voryba1,vorzba1);

        selected_index1.clear();

        selected_index1 = GetSelectedIndex(hits);

        Selection_destroy();

        ui->widget1->updateGL();
    }
    if(cloud_selected == 2)
    {
        if(cloud2->points.empty())
            return;

        Selection_init(cloud2);
        int hits = DoOpenGLSelection(press_x,press_y,release_x,release_y,cloud2,vorxba2,voryba2,vorzba2);

        selected_index2.clear();

        selected_index2 = GetSelectedIndex(hits);

        Selection_destroy();

        ui->widget1->updateGL();
    }

}

void registration::keyPressEvent(QKeyEvent *event)
{
    if(!selectMode)
        return;

    if(cloud_selected == 1)
    {
        if(Qt::Key_P == event->key())
        {
            if(!selected_index1.size())
                return;
            pick1_cloud->points.clear();
            for(unsigned int i=0; i<selected_index1.size(); i++)
            {
                pcl::PointXYZ p;
                p.x = cloud1->points[selected_index1[i]].x;
                p.y = cloud1->points[selected_index1[i]].y;
                p.z = cloud1->points[selected_index1[i]].z;
                pick1_cloud->points.push_back(p);
            }
            ui->selected_mode_Edit->setText("已保存");
        }

    }
    if(cloud_selected == 2)
    {
        if(Qt::Key_P == event->key())
        {
            if(!selected_index2.size())
                return;
            pick2_cloud->points.clear();
            for(unsigned int i=0; i<selected_index2.size(); i++)
            {
                pcl::PointXYZ p;
                p.x = cloud2->points[selected_index2[i]].x;
                p.y = cloud2->points[selected_index2[i]].y;
                p.z = cloud2->points[selected_index2[i]].z;
                pick2_cloud->points.push_back(p);
            }
            ui->selected_mode_Edit->setText("已保存");
        }

    }


}

bool registration::loadlasFile(const QString &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud, int seq_num)
{
    QByteArray fn=(fileName).toLocal8Bit();
    const char* file_name=fn.data();
    cout<<file_name<<endl;

    //ifs = NULL;
    p_cloud->points.clear();

    ifs.open(file_name, ios::in | ios::binary);
//    if (ifs == NULL)
//    {
//        cout<<"File Error!"<<endl;
//        //return 0;
//        exit(1);
//    }
    liblas::ReaderFactory f ;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();

    int point_num = header.GetPointRecordsCount();
    cout<<"Number of point records     : "<<point_num<<endl;          //记录的点数信息,下面的信息比较重要，都列出来了，具体意思也比较好理解

    cout<<"X scale factor              : "<<header.GetScaleX()<<endl;
    cout<<"Y scale factor              : "<<header.GetScaleY()<<endl;
    cout<<"Z scale factor              : "<<header.GetScaleZ()<<endl;
    cout<<"X offset                    : "<<header.GetOffsetX()<<endl;
    cout<<"Y offset                    : "<<header.GetOffsetY()<<endl;
    cout<<"Z offset                    : "<<header.GetOffsetZ()<<endl;
    cout<<"Max X                       : "<<header.GetMaxX()<<endl;
    cout<<"Max Y                       : "<<header.GetMaxY()<<endl;
    cout<<"Max Z                       : "<<header.GetMaxZ()<<endl;
    cout<<"Min X                       : "<<header.GetMinX()<<endl;
    cout<<"Min Y                       : "<<header.GetMinY()<<endl;
    cout<<"Min Z                       : "<<header.GetMinZ()<<endl;


    while(reader.ReadNextPoint())
    {
        pcl::PointXYZ p;
        p.x = reader.GetPoint().GetX();
        p.y = reader.GetPoint().GetY();
        p.z = reader.GetPoint().GetZ();
        p_cloud->points.push_back(p);

    }

    if(1 == seq_num)
    {
        max_x1 = header.GetMaxX()+0.05;
        max_y1 = header.GetMaxY()+0.05;
        max_z1 = header.GetMaxZ()+0.05;
        min_x1 = header.GetMinX()-0.05;
        min_y1 = header.GetMinY()-0.05;
        min_z1 = header.GetMinZ()-0.05;
        vorxba1 = (max_x1+min_x1)/2;
        voryba1 = (max_y1+min_y1)/2;
        vorzba1 = (max_z1+min_z1)/2;
        ui->lineEdit1->setText("点云1");
        ui->widget1->updateGL();
    }
    else if(2 == seq_num)
    {
        max_x2 = header.GetMaxX()+0.05;
        max_y2 = header.GetMaxY()+0.05;
        max_z2 = header.GetMaxZ()+0.05;
        min_x2 = header.GetMinX()-0.05;
        min_y2 = header.GetMinY()-0.05;
        min_z2 = header.GetMinZ()-0.05;
        vorxba2 = (max_x2+min_x2)/2;
        voryba2 = (max_y2+min_y2)/2;
        vorzba2 = (max_z2+min_z2)/2;
        ui->lineEdit2->setText("点云2");
        ui->widget2->updateGL();
    }
    ifs.close();
    return 1;

}
void registration::draw_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud, int seq_num)
{
    if(seq_num == 1)
    {
        glPointSize(2.0);
        glBegin( GL_POINTS);
        float boundingBoxZ=max_z1-min_z1;
        for(unsigned int i=0;i<p_cloud->points.size();i++)
        {
            float highZ=p_cloud->points[i].z-min_z1;

            if(highZ<(boundingBoxZ/3))
            {
                glColor3f(0.1f+0.7f*highZ/(boundingBoxZ/3),0.1f,0.1f);
            }
            else if(highZ<2*(boundingBoxZ/3))
            {
                glColor3f(0.8f,0.7f*(highZ-(boundingBoxZ/3))/(boundingBoxZ/3),0.1f);
            }
            else
            {
                glColor3f(0.8f,0.8f,0.1f+0.7f*(highZ-2*(boundingBoxZ/3))/(boundingBoxZ/3));
            }
            glVertex3f(p_cloud->points[i].x-vorxba1,p_cloud->points[i].y-voryba1,p_cloud->points[i].z-vorzba1);

        }
        glEnd();

        glPointSize(3.0);
        glBegin( GL_POINTS);
        glColor3f(1.0f,0.0f,0.0f);
        for(unsigned int i=0;i<selected_index1.size();i++)
            glVertex3f(p_cloud->points[selected_index1[i]].x-vorxba1,p_cloud->points[selected_index1[i]].y-voryba1,p_cloud->points[selected_index1[i]].z-vorzba1);
        glEnd();
    }
    else if(seq_num == 2)
    {
        glPointSize(2.0);
        glBegin( GL_POINTS);
        float boundingBoxZ=max_z2-min_z2;
        for(unsigned int i=0;i<p_cloud->points.size();i++)
        {
            float highZ=p_cloud->points[i].z-min_z2;

            if(highZ<(boundingBoxZ/3))
            {
                glColor3f(0.1f+0.7f*highZ/(boundingBoxZ/3),0.1f,0.1f);
            }
            else if(highZ<2*(boundingBoxZ/3))
            {
                glColor3f(0.8f,0.7f*(highZ-(boundingBoxZ/3))/(boundingBoxZ/3),0.1f);
            }
            else
            {
                glColor3f(0.8f,0.8f,0.1f+0.7f*(highZ-2*(boundingBoxZ/3))/(boundingBoxZ/3));
            }
            glVertex3f(p_cloud->points[i].x-vorxba2,p_cloud->points[i].y-voryba2,p_cloud->points[i].z-vorzba2);

        }
        glEnd();

        glPointSize(3.0);
        glBegin( GL_POINTS);
        glColor3f(1.0f,0.0f,0.0f);
        for(unsigned int i=0;i<selected_index2.size();i++)
            glVertex3f(p_cloud->points[selected_index2[i]].x-vorxba2,p_cloud->points[selected_index2[i]].y-voryba2,p_cloud->points[selected_index2[i]].z-vorzba2);
        glEnd();
    }


}

void registration::draw_selected_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud, int seq_num)
{
    if(seq_num == 1)
    {
        glPointSize(2.0);
        glBegin( GL_POINTS);
        float boundingBoxZ=max_z1-min_z1;

        for(unsigned int i=0;i<p_cloud->points.size();i++)
        {
            float highZ=p_cloud->points[i].z-min_z1;

            if(highZ<(boundingBoxZ/3))
            {
                glColor3f(0.1f+0.7f*highZ/(boundingBoxZ/3),0.1f,0.1f);
            }
            else if(highZ<2*(boundingBoxZ/3))
            {
                glColor3f(0.8f,0.7f*(highZ-(boundingBoxZ/3))/(boundingBoxZ/3),0.1f);
            }
            else
            {
                glColor3f(0.8f,0.8f,0.1f+0.7f*(highZ-2*(boundingBoxZ/3))/(boundingBoxZ/3));
            }
            glVertex3f(p_cloud->points[i].x-vorxba1,p_cloud->points[i].y-voryba1,p_cloud->points[i].z-vorzba1);

        }
        glEnd();
    }
    else if(seq_num == 2)
    {
        glPointSize(2.0);
        glBegin( GL_POINTS);
        float boundingBoxZ=max_z2-min_z2;

        for(unsigned int i=0;i<p_cloud->points.size();i++)
        {
            float highZ=p_cloud->points[i].z-min_z2;

            if(highZ<(boundingBoxZ/3))
            {
                glColor3f(0.1f+0.7f*highZ/(boundingBoxZ/3),0.1f,0.1f);
            }
            else if(highZ<2*(boundingBoxZ/3))
            {
                glColor3f(0.8f,0.7f*(highZ-(boundingBoxZ/3))/(boundingBoxZ/3),0.1f);
            }
            else
            {
                glColor3f(0.8f,0.8f,0.1f+0.7f*(highZ-2*(boundingBoxZ/3))/(boundingBoxZ/3));
            }
            glVertex3f(p_cloud->points[i].x-vorxba2,p_cloud->points[i].y-voryba2,p_cloud->points[i].z-vorzba2);

        }
        glEnd();
    }

}

void registration::Drawcloud1()
{
    if(cloud_selected == 1)
    {
        if(switch_cloud1_flag == 0)
        {
            draw_input_cloud(cloud1,1);
        }
        else if(switch_cloud1_flag == 1)
        {
            draw_selected_cloud(pick1_cloud,1);
        }
    }
    else if(cloud_selected == 2)
    {
        if(switch_cloud1_flag == 0)
        {
            draw_input_cloud(cloud2,2);
        }
        else if(switch_cloud1_flag == 1)
        {
           draw_selected_cloud(pick2_cloud,2);
        }

    }

}

void registration::Drawcloud2()
{
    if(cloud_selected == 1)
    {
        if(switch_cloud2_flag == 0)
        {
            draw_input_cloud(cloud2,2);
        }
        else if(switch_cloud2_flag == 1)
        {
            draw_selected_cloud(pick2_cloud,2);
        }
    }
    else if(cloud_selected == 2)
    {
        if(switch_cloud2_flag == 0)
        {
            draw_input_cloud(cloud1,1);
        }
        else if(switch_cloud2_flag == 1)
        {
           draw_selected_cloud(pick1_cloud,1);
        }

    }

}

void registration::Drawcloud3()
{
    if(switch_cloud3_flag == 0)
    {
        glPointSize(2.0);
        glBegin( GL_POINTS);
        glColor3f(0.0,0.0,1.0);

        for(unsigned int i=0;i<cloud_icp->points.size();i++)
            glVertex3f(cloud_icp->points[i].x-vorxba1,cloud_icp->points[i].y-voryba1,cloud_icp->points[i].z-vorzba1);

        glEnd();

        draw_selected_cloud(pick1_cloud,1);
    }
    else if(switch_cloud3_flag == 1)
    {
        glPointSize(2.0);
        glBegin( GL_POINTS);
        float boundingBoxZ=max_z3-min_z3;

        for(unsigned int i=0;i<cloud3->points.size();i++)
        {
            float highZ=cloud3->points[i].z-min_z3;

            if(highZ<(boundingBoxZ/3))
            {
                glColor3f(0.1f+0.7f*highZ/(boundingBoxZ/3),0.1f,0.1f);
            }
            else if(highZ<2*(boundingBoxZ/3))
            {
                glColor3f(0.8f,0.7f*(highZ-(boundingBoxZ/3))/(boundingBoxZ/3),0.1f);
            }
            else
            {
                glColor3f(0.8f,0.8f,0.1f+0.7f*(highZ-2*(boundingBoxZ/3))/(boundingBoxZ/3));
            }
            glVertex3f(cloud3->points[i].x-vorxba3,cloud3->points[i].y-voryba3,cloud3->points[i].z-vorzba3);

        }
        glEnd();
    }

}

void registration::on_return_Btn_clicked()
{
    window_selected = 0;

    selectMode = 0;

    cloud_selected = 1;

    cloud1->points.clear();
    cloud2->points.clear();
    cloud3->points.clear();
    selected_index1.clear();
    selected_index2.clear();
    pick1_cloud->points.clear();
    pick2_cloud->points.clear();
    cloud_icp->points.clear();

    iterations = 10;
    switch_cloud1_flag = 0;
    switch_cloud2_flag = 0;
    switch_cloud3_flag = 0;

    vorxba1 = 0,voryba1 = 0,vorzba1 = 0;
    max_x1 = -10000,min_x1 = 10000,max_y1 = -10000,min_y1 = 10000,max_z1 = -10000,min_z1 = 10000;

    vorxba2 = 0,voryba2 = 0,vorzba2 = 0;
    max_x2 = -10000,min_x2 = 10000,max_y2 = -10000,min_y2 = 10000,max_z2 = -10000,min_z2 = 10000;

    vorxba3 = 0,voryba3 = 0,vorzba3 = 0;
    max_x3 = -10000,min_x3 = 10000,max_y3 = -10000,min_y3 = 10000,max_z3 = -10000,min_z3 = 10000;

    parentWidget()->show();
    this->hide();
}

void registration::on_input1_Btn_clicked()
{
    QString fileName=QFileDialog::getOpenFileName(this,QString("open data1"));
    if(fileName == NULL)
        return;
    loadlasFile(fileName,cloud1,1);
}

void registration::on_input2_Btn_clicked()
{
    QString fileName=QFileDialog::getOpenFileName(this,QString("open data2"));
    if(fileName == NULL)
        return;
    loadlasFile(fileName,cloud2,2);
}



void registration::on_switch_Btn_clicked()
{
    cloud_selected++;
    if(cloud_selected == 3)
    {
        ui->lineEdit1->setText("点云1");
        ui->lineEdit2->setText("点云2");
        cloud_selected = 1;
    }
    else
    {
        ui->lineEdit1->setText("点云2");
        ui->lineEdit2->setText("点云1");
    }
    ui->widget1->updateGL();
    ui->widget2->updateGL();
}


void registration::on_pick_Btn_clicked()
{
    selectMode++;
    selectMode = selectMode%2;
    if(selectMode)
        ui->selected_mode_Edit->setText("选择模式");
    else
        ui->selected_mode_Edit->setText("非选择模式");
}

void registration::on_switch1_Btn_clicked()
{
    switch_cloud1_flag++;
    switch_cloud1_flag = switch_cloud1_flag%2;
    ui->widget1->updateGL();
}

void registration::on_switch2_Btn_clicked()
{
    switch_cloud2_flag++;
    switch_cloud2_flag = switch_cloud2_flag%2;
    ui->widget2->updateGL();
}

void registration::print4x4Matrix (const Eigen::Matrix4d & matrix)    //打印旋转矩阵和平移矩阵
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void registration::on_start_Btn_clicked()
{
    if(pick1_cloud->points.empty() || pick2_cloud->points.empty())
        return;



    *cloud_icp = *pick2_cloud;

    icp.setMaximumIterations (iterations);

    icp.setInputSource (cloud_icp);   //设置输入的点云
    icp.setInputTarget (pick1_cloud);    //目标点云
    icp.align (*cloud_icp);          //匹配后源点云

    if (icp.hasConverged ())//icp.hasConverged ()=1（true）输出变换矩阵的适合性评估
    {
        cout << "\nICP has converged, score is " << icp.getFitnessScore () << endl;
        cout << "\nICP transformation " << iterations << " : cloud_icp -> pick1_cloud" << endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);
        ui->widget3->updateGL();
    }
    else
    {
        printf ("\nICP has not converged.\n");
        return;
    }

    icp.setMaximumIterations (1);  // 设置为1以便下次调用

}

void registration::on_continune_Btn_clicked()
{
    if(cloud_icp->points.empty())
        return;
    icp.align(*cloud_icp);
    if (icp.hasConverged ())
    {
        printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
        cout << "\nICP transformation " << ++iterations << " : cloud_icp -> pick1_cloud" << endl;
        transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate!
        print4x4Matrix (transformation_matrix);  // 打印矩阵变换
        ui->widget3->updateGL();
    }
    else
    {
        printf ("\nICP has not converged.\n");
        return;
    }
}

void registration::on_show_all_Btn_clicked()
{
    switch_cloud3_flag++;
    switch_cloud3_flag = switch_cloud3_flag%2;
    if(switch_cloud3_flag == 1)
    {
        max_x3 = -10000,min_x3 = 10000,max_y3 = -10000,min_y3 = 10000,max_z3 = -10000,min_z3 = 10000;

        cloud3->points.clear();
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud2,*trans_cloud,transformation_matrix);
        *cloud3 = *cloud1 + *trans_cloud;

        for(unsigned int i=0; i<cloud3->points.size(); i++)
        {
            min_x3 = (cloud3->points[i].x<min_x3)?cloud3->points[i].x:min_x3;
            min_y3 = (cloud3->points[i].y<min_y3)?cloud3->points[i].y:min_y3;
            min_z3 = (cloud3->points[i].z<min_z3)?cloud3->points[i].z:min_z3;
            max_x3 = (cloud3->points[i].x>max_x3)?cloud3->points[i].x:max_x3;
            max_y3 = (cloud3->points[i].y>max_y3)?cloud3->points[i].y:max_y3;
            max_z3 = (cloud3->points[i].z>max_z3)?cloud3->points[i].z:max_z3;
        }
        vorxba3 = (max_x3+min_x3)/2;
        voryba3 = (max_y3+min_y3)/2;
        vorzba3 = (max_z3+min_z3)/2;
    }
    ui->widget3->updateGL();
}

void registration::on_save_Btn_clicked()
{
    if(cloud3->points.empty())
        return;

    char test_inlab[512];
    char filetxtname[512];
    char filelasname[512];
    time_t now;
    struct tm *tm_now;

    time(&now);
    tm_now = localtime(&now);

    sprintf(test_inlab,"/home/longmen/laser_data/%d-%d-%d_%d_%d_%d",(1900+tm_now->tm_year),(1+tm_now->tm_mon),tm_now->tm_mday,tm_now->tm_hour,tm_now->tm_min,tm_now->tm_sec);
    strcpy(filetxtname,test_inlab);
    strcpy(filelasname,test_inlab);
    strcat(filetxtname,"_registration_point.txt");
    strcat(filelasname,"_registration_point.las");

    FILE* p1=NULL;
    p1 = fopen(filetxtname,"a+");
    for(unsigned int i=0; i<cloud3->points.size(); i++)
    {
        fprintf(p1,"%lf %lf %lf \n",cloud3->points[i].x,cloud3->points[i].y,cloud3->points[i].z);
    }
    fclose(p1);

    char str[512];
    sprintf(str,"gnome-terminal -x bash -c \"txt2las %s -o %s; exec bash\"",filetxtname,filelasname);

    QProcess *pro=new QProcess();
    pro->start(str);
}

#include "filterwindow.h"
#include "mainwindow.h"

pcl::PointCloud<pcl::PointXYZ> cloud_unfilted;
pcl::PointCloud<pcl::PointXYZ> cloud_filted;
pcl::PointCloud<pcl::PointXYZ> cloud_RadFilted;
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);//点和法向量

Contour allcontourlines;
const int ContourElevationNum=3;//set it for getting contour lines
int contour_flag=0;

int fil_flag=0;
long FiniteTriNum=0;
vector<float>tra1x;
vector<float>tra1y;
vector<float>tra1z;
vector<float>tra2x;
vector<float>tra2y;
vector<float>tra2z;
vector<float>tra3x;
vector<float>tra3y;
vector<float>tra3z;

//float legAngle[2]={0.0f,0.0f};
//float armAngle[2]={0.0f,0.0f};
//float ambientLight[]={0.3f,0.5f,0.8f,1.0f};  //环境光
//float diffuseLight[]={0.5f,0.5f,0.5f,1.0f}; //散射光
//float lightPosition[]={vorxba,voryba,vorzba+20.0,1.0f}; //光源位置
//float matAmbient[]={1.0f,1.0f,1.0f,1.0f};
//float matDiff[]={1.0f,1.0f,1.0f,1.0f};

FilterWindow::FilterWindow(QWidget *parent) : QMainWindow(parent),ui(new Ui::FilterWindow)
{
    ui->setupUi(this);
}

FilterWindow::~FilterWindow()
{
    delete ui;
}

void FilterWindow::on_RetuenBtn_clicked()
{
    ColorMode = 2;
    selectMode = 2;
    vorxba = 0;voryba = 0;vorzba = 0;
    max_x = -10000;min_x = 10000;max_y = -10000;min_y = 10000;max_z = -10000;min_z = 10000;
    window_selected = 0;

    cloud_unfilted.points.clear();
    cloud_filted.points.clear();

    fil_flag=0;
    FiniteTriNum=0;
    tra1x.clear();
    tra1y.clear();
    tra1z.clear();
    tra2x.clear();
    tra2y.clear();
    tra2z.clear();
    tra3x.clear();
    tra3y.clear();
    tra3z.clear();

    contour_flag=0;

    parentWidget()->show();
    this->hide();
}

void FilterWindow::on_input_Btn_clicked()
{
    QString fileName=QFileDialog::getOpenFileName(this,QString("open data"));
    if(fileName == NULL)
        return;

    loadlasFile(fileName);
}

bool FilterWindow::loadlasFile(const QString &fileName)
{
    QByteArray fn=(fileName).toLocal8Bit();
    const char* file_name=fn.data();
    cout<<file_name<<endl;

    cloud_unfilted.points.clear();
    cloud_filted.points.clear();

    ifstream ifs;

    ifs.open(file_name, ios::in | ios::binary);

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
    cout<<"Max Y                       : "<<header.GetMaxY()<<endl;     cout<<"Max Z                       : "<<header.GetMaxZ()<<endl;
    cout<<"Min X                       : "<<header.GetMinX()<<endl;
    cout<<"Min Y                       : "<<header.GetMinY()<<endl;
    cout<<"Min Z                       : "<<header.GetMinZ()<<endl;

    max_x = header.GetMaxX()+0.05;
    max_y = header.GetMaxY()+0.05;
    max_z = header.GetMaxZ()+0.05;
    min_x = header.GetMinX()-0.05;
    min_y = header.GetMinY()-0.05;
    min_z = header.GetMinZ()-0.05;

    while(reader.ReadNextPoint())
    {
        pcl::PointXYZ p;
        p.x = reader.GetPoint().GetX();
        p.y = reader.GetPoint().GetY();
        p.z = reader.GetPoint().GetZ();
        cloud_unfilted.points.push_back(p);
    }

    vorxba = (max_x+min_x)/2;
    voryba = (max_y+min_y)/2;
    vorzba = (max_z+min_z)/2;

    ifs.close();

    ui->widget->updateGL();
    return 1;
}

void FilterWindow::Draw_unfilted_Points()
{
    glPointSize(2.0);
    glBegin( GL_POINTS);
    float boundingBoxZ=max_z-min_z;

    for(unsigned int i=0;i<cloud_unfilted.points.size();i++)
    {
        float highZ=cloud_unfilted.points[i].z-min_z;
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
        glVertex3f(cloud_unfilted.points[i].x-vorxba,cloud_unfilted.points[i].y-voryba,cloud_unfilted.points[i].z-vorzba);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(min_x-vorxba,min_y-voryba,min_z-vorzba);
    glVertex3f(max_x-vorxba,min_y-voryba,min_z-vorzba);
    glVertex3f(max_x-vorxba,max_y-voryba,min_z-vorzba);
    glVertex3f(min_x-vorxba,max_y-voryba,min_z-vorzba);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(min_x-vorxba,min_y-voryba,max_z-vorzba);
    glVertex3f(max_x-vorxba,min_y-voryba,max_z-vorzba);
    glVertex3f(max_x-vorxba,max_y-voryba,max_z-vorzba);
    glVertex3f(min_x-vorxba,max_y-voryba,max_z-vorzba);
    glEnd();
}

void FilterWindow::Draw_filted_Points()
{
    glPointSize(2.0);
    glBegin( GL_POINTS);

    float boundingBoxZ=max_z-min_z;
    for(unsigned int i=0;i<cloud_filted.points.size();i++)
    {
        float highZ=cloud_filted.points[i].z-min_z;

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
        glVertex3f(cloud_filted.points[i].x-vorxba,cloud_filted.points[i].y-voryba,cloud_filted.points[i].z-vorzba);
    }
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(min_x-vorxba,min_y-voryba,min_z-vorzba);
    glVertex3f(max_x-vorxba,min_y-voryba,min_z-vorzba);
    glVertex3f(max_x-vorxba,max_y-voryba,min_z-vorzba);
    glVertex3f(min_x-vorxba,max_y-voryba,min_z-vorzba);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(min_x-vorxba,min_y-voryba,max_z-vorzba);
    glVertex3f(max_x-vorxba,min_y-voryba,max_z-vorzba);
    glVertex3f(max_x-vorxba,max_y-voryba,max_z-vorzba);
    glVertex3f(min_x-vorxba,max_y-voryba,max_z-vorzba);
    glEnd();

}

void FilterWindow::Draw_triangulations()
{
     float boundingBoxZ=max_z-min_z;

//     glShadeModel(GL_SMOOTH);
//     glEnable(GL_LIGHTING);
//     glMaterialfv(GL_FRONT,GL_AMBIENT,matAmbient);
//     glMaterialfv(GL_FRONT,GL_DIFFUSE,matDiff);
//     //现在开始调协LIGHT0
//     glLightfv(GL_LIGHT0,GL_AMBIENT,ambientLight); //设置环境光分量
//     glLightfv(GL_LIGHT0,GL_DIFFUSE,diffuseLight); //设置散射光分量
//     glLightfv(GL_LIGHT0,GL_POSITION,lightPosition); //设置光源在场景中的位置
//     glEnable(GL_LIGHT0);
//     glEnable(GL_COLOR_MATERIAL);
     glShadeModel(GL_SMOOTH);
     glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
     glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
     glEnable(GL_LIGHT0);
     glEnable(GL_LIGHTING);
     glEnable(GL_COLOR_MATERIAL);

     glBegin( GL_TRIANGLES );
     for(int i=0;i<FiniteTriNum;i++)
     {
         float abx=tra2x[i]-tra1x[i];         //计算法线 叉乘
         float aby=tra2y[i]-tra1y[i];
         float abz=tra2z[i]-tra1z[i];

         float bcx=tra3x[i]-tra1x[i];
         float bcy=tra3y[i]-tra1y[i];
         float bcz=tra3z[i]-tra1z[i];

         float normx=(aby*bcz)-(abz*bcy);
         float normy=(abz*bcx)-(abx*bcz);
         float normz=(abx*bcy)-(aby*bcx);

         double dNormalLength=sqrt(normx*normx+normy*normy+normz*normz);         //单位化
         if(dNormalLength!=0.0)
         {
             normx=normx/dNormalLength;
             normy=normy/dNormalLength;
             normz=normz/dNormalLength;
         }
         else
         {
             normx=0.0;
             normy=0.0;
             normz=0.0;
         }
         glNormal3d(normx,normy,normz);

         float highZ;
         highZ=tra1z[i]-min_z;
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
         glVertex3f(tra1x[i]-vorxba, tra1y[i]-voryba, tra1z[i]-vorzba);

         highZ=tra2z[i]-min_z;
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
         glVertex3f(tra2x[i]-vorxba, tra2y[i]-voryba, tra2z[i]-vorzba);

         highZ=tra3z[i]-min_z;
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
         //glColor3f(0.0f,0.0f,1.0f);
         glVertex3f(tra3x[i]-vorxba, tra3y[i]-voryba, tra3z[i]-vorzba);
    }
    glEnd();
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
    //glDisable(GL_NORMALIZE);

    if(contour_flag==1)    //show contour
    {
        glEnable(GL_LINE_SMOOTH);
        glColor3f(0.0f,0.0f,1.0f);
        glLineWidth(3.0);

        std::multimap<Properties, ContourLine>::iterator it;
        std::vector< std::vector<ContourPoint> >::iterator it1;
        std::vector<ContourPoint>::iterator it2;

        for (it=allcontourlines.contour.begin(); it!=allcontourlines.contour.end(); ++it)
        {
               double elevation= (*it).first.elevation;
               for(it1=(*it).second.cl.begin(); it1!=(*it).second.cl.end(); ++it1)
               {
                   glBegin(GL_LINE_STRIP);
                   for(it2=(*it1).begin();it2!=(*it1).end();++it2)
                   {
                       glVertex3f( (*it2).x-vorxba,(*it2).y-voryba, elevation-vorzba);
                   }
                   glEnd();
               }
        }
        glDisable(GL_LINE_SMOOTH);
    }

    glBegin(GL_LINE_LOOP);
    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(min_x-vorxba,min_y-voryba,min_z-vorzba);
    glVertex3f(max_x-vorxba,min_y-voryba,min_z-vorzba);
    glVertex3f(max_x-vorxba,max_y-voryba,min_z-vorzba);
    glVertex3f(min_x-vorxba,max_y-voryba,min_z-vorzba);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(min_x-vorxba,min_y-voryba,max_z-vorzba);
    glVertex3f(max_x-vorxba,min_y-voryba,max_z-vorzba);
    glVertex3f(max_x-vorxba,max_y-voryba,max_z-vorzba);
    glVertex3f(min_x-vorxba,max_y-voryba,max_z-vorzba);
    glEnd();
    

}

void FilterWindow::on_save_Btn_clicked()
{
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
    strcat(filetxtname,"_filted_point.txt");
    strcat(filelasname,"_filted_point.las");

    FILE* p1=NULL;
    p1 = fopen(filetxtname,"a+");
    for(unsigned int i=0; i<cloud_filted.points.size(); i++)
    {
        fprintf(p1,"%lf %lf %lf \n",cloud_filted.points[i].x,cloud_filted.points[i].y,cloud_filted.points[i].z);
    }
    fclose(p1);

    char str[512];
    sprintf(str,"gnome-terminal -x bash -c \"txt2las %s -o %s; exec bash\"",filetxtname,filelasname);

    QProcess *pro=new QProcess();
    pro->start(str);
}

void FilterWindow::on_DEM_Btn_clicked()
{
    long point_num=cloud_filted.points.size();
    FiniteTriNum=0;
    PointNode *pointc=new PointNode[point_num];
    for(int i=0;i<point_num;i++)
    {
        PointStorage_create_point(pointc+i, cloud_filted.points[i].x,
                                            cloud_filted.points[i].y,
                                            cloud_filted.points[i].z);
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
    std::cout<< "TIN size is " <<TINget_size()<< std::endl;

    TINtriangle *output;
    for (int i=0; i<TINget_size();i++)
    {
         output=TINget_triangle(i);
         if(output->V[0])
         {
/*          std::cout<<"This is finite triangle:"<<std::endl;
                std::cout<<"Neibor0_tri: "<<TIN_TRIANGLE(output->N[0])<<"  "
                         <<"Neibor0_cor: "<<TIN_CORNER(output->N[0])<<std::endl;
                std::cout<<"Neibor1_tri: "<<TIN_TRIANGLE(output->N[1])<<"  "
                         <<"Neibor1_cor: "<<TIN_CORNER(output->N[1])<<std::endl;
                std::cout<<"Neibor2_tri: "<<TIN_TRIANGLE(output->N[2])<<"  "
                         <<"Neibor3_cor: "<<TIN_CORNER(output->N[2])<<std::endl;
                std::cout<<"Vertex0: ("<<*(output->V[0])<<", "<<*(output->V[0]+1)<<", "<<*(output->V[0]+2)<<")"<<std::endl;
                std::cout<<"Vertex1: ("<<*(output->V[1])<<", "<<*(output->V[1]+1)<<", "<<*(output->V[1]+2)<<")"<<std::endl;
                std::cout<<"Vertex2: ("<<*(output->V[2])<<", "<<*(output->V[2]+1)<<", "<<*(output->V[2]+2)<<")"<<std::endl;*/

            tra1x.push_back(*(output->V[0]));
            tra1y.push_back(*(output->V[0]+1));
            tra1z.push_back(*(output->V[0]+2));
            tra2x.push_back(*(output->V[1]));
            tra2y.push_back(*(output->V[1]+1));
            tra2z.push_back(*(output->V[1]+2));
            tra3x.push_back(*(output->V[2]));
            tra3y.push_back(*(output->V[2]+1));
            tra3z.push_back(*(output->V[2]+2));
            FiniteTriNum++;
          }
          else
          {
/*           std::cout<<"This is infinite triangle:"<<std::endl;
                std::cout<<"Neibor0_tri: "<<TIN_TRIANGLE(output->N[0])<<"  "
                         <<"Neibor0_cor: "<<TIN_CORNER(output->N[0])<<std::endl;
                //std::cout<<"Neibor1_tri: "<<TIN_TRIANGLE(output->N[1])<<"  "
                //         <<"Neibor1_cor: "<<TIN_CORNER(output->N[1])<<std::endl;
                //std::cout<<"Neibor2_tri: "<<TIN_TRIANGLE(output->N[2])<<"  "
                //         <<"Neibor2_cor: "<<TIN_CORNER(output->N[2])<<std::endl;
                std::cout<<"Vertex1: ("<<*(output->V[1])<<", "<<*(output->V[1]+1)<<", "<<*(output->V[1]+2)<<")"<<std::endl;
                std::cout<<"Vertex2: ("<<*(output->V[2])<<", "<<*(output->V[2]+1)<<", "<<*(output->V[2]+2)<<")"<<std::endl;*/
          }
    }
    delete []pointc;
    ui->widget->updateGL();
}

void FilterWindow::on_conture_Btn_clicked()
{
     long point_num=cloud_filted.points.size();
     PointNode *pointc=new PointNode[point_num];
     for(int i=0;i<point_num;i++)
     {
         PointStorage_create_point(pointc+i, cloud_filted.points[i].x,
                                             cloud_filted.points[i].y,
                                             cloud_filted.points[i].z);
     }

     float Zmax=max_z-0.05;
     float Zmin=min_z+0.05;

     //Preprocess the cloud points for getting contour lines
     SlightChange(pointc, point_num, Zmin, Zmax, ContourElevationNum);

     int count = 0;
     fprintf(stderr, "computing TIN ... \n");
     TINclean(point_num, 0);				//???TODO  problem , but seems no bugs...
     printf("tin, point num  =%d!!!\n", point_num);
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
     std::cout<< "TIN size is " <<TINget_size()<< std::endl;
     //Contour tracking starts.
     TINtriangle *t=TINget_triangle(0);
     const long TinNum=TINget_size();
     Flag flag[TinNum];
     getAllContourLines(t, flag, allcontourlines, Zmin, Zmax, ContourElevationNum, TinNum);
     contour_flag=1;
     delete []pointc;
     ui->widget->updateGL();
}

void FilterWindow::on_smooth_Btn_clicked()//平滑滤波
{
    if(cloud_unfilted.points.empty())
        return;
    cloud_filted.points.clear();

    float WIDTH = 0.3;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (WIDTH);

    octree.defineBoundingBox(min_x,min_y,min_z,max_x,max_y,max_z);
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
                double temp_min_z = 100;
                std::vector<Point> temp_point_vec;

                for(searchPoint.z = min_z;searchPoint.z<=max_z;searchPoint.z+=WIDTH)
                {
                    std::vector<int> pointIdxVec;
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

                            cloud_filted.points.push_back(point_selected);

                        }
                    }
                }
            }
        }
    ui->widget->updateGL();
}

void FilterWindow::on_curvature_Btn_clicked()//曲率滤波
{
    cloud_filted.points.clear();    //去除离群点
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_unfilted.makeShared());
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius(2);
    outrem.filter(cloud_RadFilted);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;    //法向计算
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_RadFilted.makeShared());
    n.setInputCloud (cloud_RadFilted.makeShared());
    n.setSearchMethod (tree);
    //n.setRadiusSearch (10);
    n.setKSearch(50);
    n.compute (*normals);
    pcl::concatenateFields(cloud_RadFilted, *normals, *cloud_with_normals);
    //    for(int i=0;i<cloud_with_normals->width;i++)
    //    {
    //        //cout<<cloud_with_normals->points[i]<<endl;//输出三个法向量
    //        //cout<<cloud_with_normals->points[i].curvature<<endl;//输出曲率
    //        //cout<<normals->points[i]<<endl;//输出三个法向量
    //        //cout<<"PointsNum:"<<cloud_with_normals->width<<endl;//输出点云数量
    //        //cout<<*cloud_with_normals;//输出PCL 法向量信息
    //    }

    //    for(int i=0;i<cloud_with_normals->width;i++)
    //    {
    //        if(cloud_with_normals->points[i].normal_z>0)
    //        {
    //            cloud_with_normals->points[i].normal_x=-cloud_with_normals->points[i].normal_x;
    //            cloud_with_normals->points[i].normal_y=-cloud_with_normals->points[i].normal_y;
    //            cloud_with_normals->points[i].normal_z=-cloud_with_normals->points[i].normal_z;
    //        }
    //    }

    float WIDTH = 0.3;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (WIDTH);
    octree.defineBoundingBox(min_x,min_y,min_z,max_x,max_y,max_z);
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

                //float temp_min_cur = 100;
                //float temp_max_cur = -100;
                if (octree.voxelSearch (searchPoint, pointIdxVec))
                {
                    float vari=0;
                    for (size_t i = 0; i < pointIdxVec.size (); ++i)
                    {
                            average_cur+=cloud_with_normals->points[pointIdxVec[i]].curvature;
                            //cur_vec.push_back(cloud_with_normals->points[pointIdxVec[i]].curvature);
                    }
                    average_cur=average_cur/pointIdxVec.size ();
                    //cout<<"temp_min_cur:"<<temp_min_cur<<" temp_max_cur"<<temp_max_cur<<" average_cur"<<average_cur<<endl;

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
                          cloud_filted.push_back(cloud_RadFilted.points[idsel]);
                    }

                    if(vari >= Svari)
                    {
                        pcl::PointXYZ p;
                        for (size_t i = 0; i < pointIdxVec.size (); i++)
                        {
 //                            if(cloud_with_normals->points[pointIdxVec[i]].curvature >= average_cur+0.03
 //                             ||cloud_with_normals->points[pointIdxVec[i]].curvature <= average_cur-0.03)

                            if((cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur)*
                               (cloud_with_normals->points[pointIdxVec[i]].curvature-average_cur) >= vari)
                            {
                                p.x = cloud_RadFilted.points[pointIdxVec[i]].x;
                                p.y = cloud_RadFilted.points[pointIdxVec[i]].y;
                                p.z = cloud_RadFilted.points[pointIdxVec[i]].z;
                                cloud_filted.push_back(p);
                            }
                        }
                    }
                }
            }
        }
    }

    fil_flag=1;
    ui->widget->updateGL();
}

void FilterWindow::on_normal_Btn_clicked()//法向滤波
{
    cloud_filted.points.clear();    //去除离群点
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
    //n.setRadiusSearch (10);
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

        int K = 50;
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
          float avenorx=dealnorx;
          float avenory=dealnory;
          float avenorz=dealnorz;

          dealPoint.z=( -ABC(0,0)*avenorx*searchPoint.z - ABC(1,0)*avenory*searchPoint.z + ABC(2,0)*avenorz + ABC(0,0)*searchPoint.x*avenorz + ABC(1,0)*searchPoint.y*avenorz)/( avenorz - ABC(0,0)*avenorx  - ABC(1,0)*avenory ) ;
          dealPoint.x=( dealPoint.z - searchPoint.z )*avenorx/avenorz + searchPoint.x;
          dealPoint.y=( dealPoint.z - searchPoint.z )*avenory/avenorz + searchPoint.y;

          //cout<<5555<<dealPoint.x<<endl<<dealPoint.y<<endl<<dealPoint.z<<endl;
          cloud_filted.push_back(dealPoint);
        }
    }
    ui->widget->updateGL();
}

void FilterWindow::on_bothside_Btn_clicked()//双边滤波
{
    cloud_filted.points.clear();    //去除离群点
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
    //n.setRadiusSearch (10);
    n.setKSearch(50);
    n.compute (*normals);
    pcl::concatenateFields(cloud_RadFilted, *normals, *cloud_with_normals);

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud (cloud_RadFilted.makeShared());
    pcl::PointXYZ searchPoint;
    float sigmac=2;
    float sigmas=0.8;
    for(int i=0;i<cloud_RadFilted.points.size();i++)
    {
        searchPoint.x = cloud_RadFilted.points[i].x;
        searchPoint.y = cloud_RadFilted.points[i].y;
        searchPoint.z = cloud_RadFilted.points[i].z;
        //cout<<"11111:"<<searchPoint.x<<"  "<<searchPoint.y<<"  "<<searchPoint.z<<endl;

        float norx=cloud_with_normals->points[i].normal_x;
        float nory=cloud_with_normals->points[i].normal_y;
        float norz=cloud_with_normals->points[i].normal_z;
        float cur=cloud_with_normals->points[i].curvature;

        int K = 50;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
  //      std::cout << "K nearest neighbor search at (" << searchPoint.x
  //                << " " << searchPoint.y
  //                << " " << searchPoint.z
  //                << ") with K=" << K << std::endl;

        float snum=0;
        float sden=0;
        float d=0;
        if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {

          for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
          {
              float dis= sqrt(pointNKNSquaredDistance[i]);
              float wsigmac=exp( -(dis*dis)/(2*sigmac*sigmac) );

              float norIAndIJ=sqrt( (norx*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x+
                      nory*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y+
                      norz*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z)*
                      (norx*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x+
                       nory*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y+
                       norz*cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z) );

              float yy=(norx-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x)*(norx-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x)+
                      (nory-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y)*(nory-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y)+
                      (norz-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z)*(norz-cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z);
              float wsigmas=exp( -yy/(2*sigmas*sigmas) );
              float nijqikij=(cloud_with_normals->points[pointIdxNKNSearch[i]].normal_x*(searchPoint.x- cloud_RadFilted.points[ pointIdxNKNSearch[i] ].x) )+
                      (cloud_with_normals->points[pointIdxNKNSearch[i]].normal_y*(searchPoint.y- cloud_RadFilted.points[ pointIdxNKNSearch[i] ].y) )+
                      (cloud_with_normals->points[pointIdxNKNSearch[i]].normal_z*(searchPoint.z- cloud_RadFilted.points[ pointIdxNKNSearch[i] ].z) );
              float num=wsigmac*dis*wsigmas*norIAndIJ*nijqikij;
              float den=wsigmac*dis*wsigmas*norIAndIJ;
              snum+=num;
              sden+=den;

//              std::cout << "    "  <<   cloud_unfilted.points[ pointIdxNKNSearch[i] ].x
//                          << " " << cloud_unfilted.points[ pointIdxNKNSearch[i] ].y
//                          << " " << cloud_unfilted.points[ pointIdxNKNSearch[i] ].z
//                          << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
          }
          d=snum/sden;
        }

        searchPoint.x+=d*norx;
        searchPoint.y+=d*nory;
        searchPoint.z+=d*norz;
        cloud_filted.push_back(searchPoint);
        //cout<<"5555:"<<searchPoint.x<<"  "<<searchPoint.y<<"  "<<searchPoint.z<<endl;
    }
    ui->widget->updateGL();
}

void FilterWindow::on_thick_Btn_clicked()//厚度滤波
{
    if(cloud_unfilted.points.empty())
        return;
    cloud_filted.points.clear();

    float WIDTH = 0.3;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (WIDTH);

    octree.defineBoundingBox(min_x,min_y,min_z,max_x,max_y,max_z);
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
                double temp_min_z = 100;
                double temp_max_z = -100;

                std::vector<Point> temp_point_vec;

                for(searchPoint.z = min_z;searchPoint.z<=max_z;searchPoint.z+=WIDTH)
                {
                    std::vector<int> pointIdxVec;
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
                        if( ( (temp_max_z-temp_min_z)>0.4)&&( (temp_max_z-temp_min_z)<1.4) )
                        {
                            if( (item->z > (averagez-0.15))&&(item->z < (averagez+0.15)) )
                            {
                                pcl::PointXYZ point_selected;
                                point_selected.x = item->x;
                                point_selected.y = item->y;
                                point_selected.z = item->z;

                                cloud_filted.points.push_back(point_selected);
                            }
                        }
                        else
                        {
                            pcl::PointXYZ point_selected;
                            point_selected.x = item->x;
                            point_selected.y = item->y;
                            point_selected.z = item->z;

                            cloud_filted.points.push_back(point_selected);
                        }
                    }
                }
            }
        }

    ui->widget->updateGL();
}

void FilterWindow::on_voxelgrid_Btn_clicked()//VoxelGrid 滤波
{
    cloud_filted.points.clear();
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_unfilted.makeShared());
    sor.setLeafSize(0.3f,0.3f,0.3f);
    sor.filter(cloud_filted);
    ui->widget->updateGL();
}

void FilterWindow::on_statistics_Btn_clicked()//统计滤波
{
    //    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //    sor.setInputCloud(cloud_unfilted.makeShared());
    //    sor.setMeanK(50);
    //    sor.setStddevMulThresh(3.0);
    //    sor.filter(cloud_filted);

    static int radious_num=0;
    radious_num = ui->Filter_Edit->text().toInt();

    cloud_filted.points.clear();    //去除离群点
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_unfilted.makeShared());
    outrem.setRadiusSearch(0.3);
    outrem.setMinNeighborsInRadius(radious_num);
    outrem.filter(cloud_filted);
    ui->widget->updateGL();
}



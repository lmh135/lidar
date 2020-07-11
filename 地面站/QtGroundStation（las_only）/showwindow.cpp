#include "showwindow.h"
#include "mainwindow.h"


int ColorMode = 0;
int selectMode = 0;
int MeasureDisMode = 0;

double vorxba = 0,voryba = 0,vorzba = 0;
float max_x = -10000,min_x = 10000,max_y = -10000,min_y = 10000,max_z = -10000,min_z = 10000;
pcl::PointCloud<pcl::PointXYZ> cloud_in;
//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (0.1);
vector<int> selected_index;
vector<Point> selected_point_save;

static int which_point = 1;
static float s_ruler_point[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

ShowWindow::ShowWindow(QWidget *parent) : QMainWindow(parent),ui(new Ui::ShowWindow)
{
    ui->setupUi(this);

    ui->xEdit->setStyleSheet("border-width:0;border-style:outset");//设置边框透明
    ui->yEdit->setStyleSheet("border-width:0;border-style:outset");//设置边框透明
    ui->zEdit->setStyleSheet("border-width:0;border-style:outset");//设置边框透明
    ui->SelectEdit->setStyleSheet("border-width:0;border-style:outset");
    ui->ColorModeEdit->setStyleSheet("border-width:0;border-style:outset");
    ui->MeasureDistanceEdit->setStyleSheet("border-width:0;border-style:outset");


    connect(ui->ColorModeBtn,SIGNAL(clicked()),this,SLOT(ChangeColorMode()));

    connect(ui->widget, SIGNAL(release_left_mouse()), this, SLOT(SelectPoint()));
}

ShowWindow::~ShowWindow()
{
    delete ui;
}

void ShowWindow::ClearMode()
{

}

void ShowWindow::ChangeColorMode()
{
    ColorMode++;
    ColorMode = ColorMode%2;


    if( ColorMode)
        ui->ColorModeEdit->setText("高程颜色模式");
    else
        ui->ColorModeEdit->setText("原始颜色模式");
    ui->widget->updateGL();
}

/*
void ShowWindow::ChangeSelectMode()
{
    selectMode=selectMode+1;
    if(selectMode>2) selectMode=1;
    if(selectMode==1)
    {
        ui->SelectEdit->setText("选点模式");
    }
    else if(selectMode==2)
    {
       ui->SelectEdit->setText("非选点模式");
       ui->xEdit->setText(" ");
       ui->yEdit->setText(" ");
       ui->zEdit->setText(" ");

    }
    ui->widget->updateGL();

}
*/

inline float sqr(float a)
{
    return a*a;
}

void ShowWindow::SelectPoint()
{
    if(cloud_in.points.empty())
        return;

    float press_x,press_y,release_x,release_y;
    press_x = (float)ui->widget->pressPos.x();
    press_y = (float)ui->widget->pressPos.y();
    release_x = (float)ui->widget->releasePos.x();
    release_y = (float)ui->widget->releasePos.y();

   // cout<<press_x<<" "<<press_y<<" "<<release_x<<" "<<release_y<<endl;
    Selection_init(cloud_in.makeShared());

    int hits = DoOpenGLSelection(press_x,press_y,release_x,release_y,cloud_in.makeShared(),vorxba,voryba,vorzba);
   // cout<<"hits: "<<hits<<endl;

    selected_index.clear();

    selected_index = GetSelectedIndex(hits);
    //cout<<"index size: "<<selected_index.size()<<endl;
/*
    if(selected_index.size())
    {
        for(int i=0; i<selected_index.size(); i++)
            cout<<i<<" : "<<selected_index[i]<<endl;
    }
*/
    Selection_destroy();

    QString text = QString::number(hits);

    ui->SelectEdit->setText("选中"+text+"个点");
    ui->xEdit->setText("");
    ui->yEdit->setText("");
    ui->zEdit->setText("");

    if(MeasureDisMode)
    {
//        static int which_point = 1;
        double ave_x,ave_y,ave_z;
        float length;
        float last_ave_x;
        int SelectEffect=0;
        for(unsigned int i=0; i<selected_index.size(); i++)
        {
            ave_x = ave_x + cloud_in.points[selected_index[i]].x;
            ave_y = ave_y + cloud_in.points[selected_index[i]].y;
            ave_z = ave_z + cloud_in.points[selected_index[i]].z;
        }
        ave_x = ave_x/selected_index.size();
        ave_y = ave_y/selected_index.size();
        ave_z = ave_z/selected_index.size();

        if( ave_x >= min_x && ave_x <= max_x ) SelectEffect=1;
        else SelectEffect=0;
        if(which_point == 1 && SelectEffect==1)
        {
            s_ruler_point[0]= ave_x;
            s_ruler_point[1]= ave_y;
            s_ruler_point[2]= ave_z;
        }
        else if(which_point == 2 && SelectEffect==1)
        {
            s_ruler_point[3]= ave_x;
            s_ruler_point[4]= ave_y;
            s_ruler_point[5]= ave_z;
            length = sqrtf(sqr(s_ruler_point[0] - s_ruler_point[3]) + sqr(s_ruler_point[1] - s_ruler_point[4]) + sqr(s_ruler_point[2] - s_ruler_point[2]));
            ui->xEdit->setText("distance: "+QString::number(length));
        }
        if(last_ave_x != ave_x && SelectEffect==1)
        {
            which_point+=1;
            if(which_point>2) which_point=1;
        }
        last_ave_x = ave_x;
        cout<<s_ruler_point[0]<<" "<<s_ruler_point[1]<<" "<<s_ruler_point[2]<<" "<<s_ruler_point[3]<<" "<<s_ruler_point[4]<<" "<<s_ruler_point[5]<<endl;
    }

    ui->widget->updateGL();
}
/*
void ShowWindow::Show_Selected_Point()
{
    QString tempStr;
    ui->xEdit->setText("X: "+tempStr.setNum(nearestPoint.x));
    ui->yEdit->setText("Y: "+tempStr.setNum(nearestPoint.y));
    ui->zEdit->setText("Z: "+tempStr.setNum(nearestPoint.z));

    ui->widget->updateGL();
}
*/

void ShowWindow::keyPressEvent(QKeyEvent *event)
{
    if(!selectMode)
        return;

   // cout<<"key:"<<event->key()<<" "<<Qt::Key_Tab<<endl;

    if(Qt::Key_Delete == event->key())//删除选中的点云
    {
        if(!selected_index.size())
            return;
        selected_point_save.clear();
        for(unsigned int i=0; i<selected_index.size(); i++)
        {
            Point p;
            p.x = cloud_in.points[selected_index[i]].x;
            p.y = cloud_in.points[selected_index[i]].y;
            p.z = cloud_in.points[selected_index[i]].z;
            selected_point_save.push_back(p);
        }
        for(unsigned int i=0; i<selected_index.size(); i++)
            cloud_in.points.erase(cloud_in.points.begin()+(selected_index[i]-i));

        selected_index.clear();
    }
    if(Qt::Key_P == event->key())//删除片选外的点云
    {
        if(!selected_index.size())
            return;
        vector<Point> p_selected;
        for(unsigned int i=0; i<selected_index.size(); i++)
        {
            Point p;
            p.x = cloud_in.points[selected_index[i]].x;
            p.y = cloud_in.points[selected_index[i]].y;
            p.z = cloud_in.points[selected_index[i]].z;
            p_selected.push_back(p);
        }
        cloud_in.points.clear();
        for(unsigned int i=0; i<p_selected.size(); i++)
        {
            pcl::PointXYZ p;
            p.x = p_selected[i].x;
            p.y = p_selected[i].y;
            p.z = p_selected[i].z;
            cloud_in.points.push_back(p);
        }
        selected_index.clear();
    }
    if(Qt::Key_S == event->key())//显示被选中第一个点的坐标
    {
        if(!selected_index.size())
            return;
        ui->xEdit->setText("X: "+QString::number(cloud_in.points[selected_index[0]].x));
        ui->yEdit->setText("Y: "+QString::number(cloud_in.points[selected_index[0]].y));
        ui->zEdit->setText("Z: "+QString::number(cloud_in.points[selected_index[0]].z));
    }
    if(Qt::Key_C == event->key())//显示被选点的平均坐标
    {
        if(!selected_index.size())
            return;
        double ave_x,ave_y,ave_z;
        for(unsigned int i=0; i<selected_index.size(); i++)
        {
            ave_x = ave_x + cloud_in.points[selected_index[i]].x;
            ave_y = ave_y + cloud_in.points[selected_index[i]].y;
            ave_z = ave_z + cloud_in.points[selected_index[i]].z;
        }
        ave_x = ave_x/selected_index.size();
        ave_y = ave_y/selected_index.size();
        ave_z = ave_z/selected_index.size();

        ui->xEdit->setText("X: "+QString::number(ave_x));
        ui->yEdit->setText("Y: "+QString::number(ave_y));
        ui->zEdit->setText("Z: "+QString::number(ave_z));
       // ui->SelectEdit->setText("选中"+text+"个点");
    }
    if(Qt::Key_B == event->key())//恢复上一步被删除的点
    {
        if(!selected_point_save.size())
            return;
        for(unsigned int i=0; i<selected_point_save.size(); i++)
        {
            pcl::PointXYZ p;
            p.x = selected_point_save[i].x;
            p.y = selected_point_save[i].y;
            p.z = selected_point_save[i].z;
            cloud_in.points.push_back(p);
        }
        selected_point_save.clear();
    }

    ui->widget->updateGL();
}

vector<Point> distance_vector;
void ShowWindow::on_Measure_distance_clicked()
{
    MeasureDisMode++;
    MeasureDisMode = MeasureDisMode%2;
    if(MeasureDisMode)
    {
        ui->MeasureDistanceEdit->setText("测距模式");
    }
    else
    {
        ui->MeasureDistanceEdit->setText("非测距模式");
    }
}

void ShowWindow::on_ReturnBtn_clicked()
{
    ColorMode = 0;
    selectMode = 0;
    MeasureDisMode = 0;

    vorxba = 0;voryba = 0;vorzba = 0;
    max_x = -10000;min_x = 10000;max_y = -10000;min_y = 10000;max_z = -10000;min_z = 10000;
    window_selected = 0;

    cloud_in.points.clear();
    selected_index.clear();
    selected_point_save.clear();

    parentWidget()->show();
    this->hide();

}

bool ShowWindow::loadlasFile(const QString &fileName)
{
    QByteArray fn=(fileName).toLocal8Bit();
    const char* file_name=fn.data();
    cout<<file_name<<endl;

    //ifs = NULL;
    cloud_in.points.clear();

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
    max_x = header.GetMaxX()+0.05;
    max_y = header.GetMaxY()+0.05;
    max_z = header.GetMaxZ()+0.05;
    min_x = header.GetMinX()-0.05;
    min_y = header.GetMinY()-0.05;
    min_z = header.GetMinZ()-0.05;




    //cloud_in.width = point_num;
   // cloud_in.height = 1;
    //cloud_in.points.resize (cloud_in.width * cloud_in.height);

    while(reader.ReadNextPoint())
    {
        pcl::PointXYZ p;
        p.x = reader.GetPoint().GetX();
        p.y = reader.GetPoint().GetY();
        p.z = reader.GetPoint().GetZ();
        cloud_in.points.push_back(p);

    }
   // vorxba=vorxba/point_num;
    //voryba=voryba/point_num;
   // vorzba=vorzba/point_num;
    vorxba = (max_x+min_x)/2;
    voryba = (max_y+min_y)/2;
    vorzba = (max_z+min_z)/2;

    //octree.deleteTree();

   // octree.defineBoundingBox(min_x,min_y,min_z,max_x,max_y,max_z);
  //  octree.setInputCloud(cloud_in.makeShared());
  //  octree.addPointsFromInputCloud();

    ifs.close();


    ui->widget->updateGL();
    ////////////////////////////////////
    return 1;

}

void ShowWindow::DrawLasPoints()
{
    glPointSize(2.0);
    glBegin( GL_POINTS);

    float boundingBoxZ=max_z-min_z;

    for(unsigned int i=0;i<cloud_in.points.size();i++)
    {
        //glPointSize(3.0);

        if(ColorMode)
        {
            float highZ=cloud_in.points[i].z-min_z;


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
            glVertex3f(cloud_in.points[i].x-vorxba,cloud_in.points[i].y-voryba,cloud_in.points[i].z-vorzba);
        }
        else
        {
            glColor3f(0.0f,0.0f,0.0f);
            glVertex3f(cloud_in.points[i].x-vorxba,cloud_in.points[i].y-voryba,cloud_in.points[i].z-vorzba);
        }
    }
    glEnd();

    glPointSize(3.0);
    glBegin( GL_POINTS);
    glColor3f(1.0f,0.0f,0.0f);
    for(unsigned int i=0;i<selected_index.size();i++)
        glVertex3f(cloud_in.points[selected_index[i]].x-vorxba,cloud_in.points[selected_index[i]].y-voryba,cloud_in.points[selected_index[i]].z-vorzba);
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

    if(MeasureDisMode && which_point==1)
    {
        glLineWidth(3.0);
        glBegin(GL_LINES);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(s_ruler_point[0]-vorxba,s_ruler_point[1]-voryba,s_ruler_point[2]-vorzba);
        glVertex3f(s_ruler_point[3]-vorxba,s_ruler_point[4]-voryba,s_ruler_point[5]-vorzba);
        glEnd();
    }

}

void ShowWindow::on_openBtn_clicked()
{
    QString fileName=QFileDialog::getOpenFileName(this,QString("open data"));
    if(fileName == NULL)
        return;
    loadlasFile(fileName);
}

void ShowWindow::on_SelectBtn_clicked()
{
    if(cloud_in.points.empty())
        return;

    selectMode++;
    selectMode = selectMode%2;
    if(selectMode)
    {
        ui->SelectEdit->setText("选点模式");
    }
    else
    {
        ui->SelectEdit->setText("非选点模式");
    }
}

void ShowWindow::on_SaveBtn_clicked()
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
    strcat(filetxtname,"_selected_point.txt");
    strcat(filelasname,"_selected_point.las");

    FILE* p1=NULL;
    p1 = fopen(filetxtname,"a+");
    for(unsigned int i=0; i<cloud_in.points.size(); i++)
    {
        fprintf(p1,"%lf %lf %lf \n",cloud_in.points[i].x,cloud_in.points[i].y,cloud_in.points[i].z);
    }
    fclose(p1);

    char str[512];
    sprintf(str,"gnome-terminal -x bash -c \"txt2las %s -o %s; exec bash\"",filetxtname,filelasname);

    QProcess *pro=new QProcess();
    pro->start(str);


}



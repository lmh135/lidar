#include "test.h"
#include "ui_test.h"

//vector<int> selected_index;
//vector<int> selected_index_save;

pcl::PointCloud<pcl::PointXYZ> test_cloud;


test::test(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::test)
{
    ui->setupUi(this);
    connect(ui->widget, SIGNAL(release_left_mouse()), this, SLOT(SelectPoint()));
}

test::~test()
{
    delete ui;
}

void test::SelectPoint()
{
    if(test_cloud.points.empty())
        return;

    float press_x,press_y,release_x,release_y;
    press_x = (float)ui->widget->pressPos.x();
    press_y = (float)ui->widget->pressPos.y();
    release_x = (float)ui->widget->releasePos.x();
    release_y = (float)ui->widget->releasePos.y();

    cout<<press_x<<" "<<press_y<<" "<<release_x<<" "<<release_y<<endl;
    Selection_init(test_cloud.makeShared());

    int hits = DoOpenGLSelection(press_x,press_y,release_x,release_y,test_cloud.makeShared(),vorxba,voryba,vorzba);
    cout<<"hits: "<<hits<<endl;

    selected_index.clear();

    selected_index = GetSelectedIndex(hits);
    cout<<"index size: "<<selected_index.size()<<endl;
/*
    if(selected_index.size())
    {
        for(int i=0; i<selected_index.size(); i++)
            cout<<i<<" : "<<selected_index[i]<<endl;
    }
*/
    Selection_destroy();
    ui->widget->updateGL();
}



void test::on_init_Btn_clicked()
{
    QString fileName=QFileDialog::getOpenFileName(this,QString("open data"));
    if(fileName == NULL)
        return;
    loadlasFile(fileName);
}

bool test::loadlasFile(const QString &fileName)
{
    QByteArray fn=(fileName).toLocal8Bit();
    const char* file_name=fn.data();
    cout<<file_name<<endl;

    //ifs = NULL;
    test_cloud.points.clear();

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
        test_cloud.points.push_back(p);

    }

    vorxba = (max_x+min_x)/2;
    voryba = (max_y+min_y)/2;
    vorzba = (max_z+min_z)/2;

    ifs.close();


    ui->widget->updateGL();
    ////////////////////////////////////
    return 1;

}

void test::on_return_Btn_clicked()
{
    window_selected = 0;
    vorxba = 0;voryba = 0;vorzba = 0;
    selectMode = 0;

    test_cloud.points.clear();
    selected_index.clear();
    selected_point_save.clear();

    parentWidget()->show();
    this->hide();
}


void test::DrawPoint()
{
    glPointSize(2.0);
    glBegin( GL_POINTS);

    glColor3f(0.0f,0.0f,0.0f);

    for(unsigned int i=0;i<test_cloud.points.size();i++)
        glVertex3f(test_cloud.points[i].x-vorxba,test_cloud.points[i].y-voryba,test_cloud.points[i].z-vorzba);
    glEnd();

    glPointSize(3.0);
    glBegin( GL_POINTS);

    glColor3f(1.0f,0.0f,0.0f);

    for(unsigned int i=0;i<selected_index.size();i++)
        glVertex3f(test_cloud.points[selected_index[i]].x-vorxba,test_cloud.points[selected_index[i]].y-voryba,test_cloud.points[selected_index[i]].z-vorzba);
    glEnd();

}

void test::keyPressEvent(QKeyEvent *event)
{
    if(selectMode == 0)
        return;

    if(Qt::Key_Delete == event->key())
    {
        for(unsigned int i=0; i<selected_index.size(); i++)
        {
            Point p;
            p.x = test_cloud.points[selected_index[i]].x;
            p.y = test_cloud.points[selected_index[i]].y;
            p.z = test_cloud.points[selected_index[i]].z;
            selected_point_save.push_back(p);
        }
        for(unsigned int i=0; i<selected_index.size(); i++)
            test_cloud.points.erase(test_cloud.points.begin()+(selected_index[i]-i));

        selected_index.clear();
    }

}


void test::on_pick_Btn_clicked()
{
    static int entry = 0;
    entry++;
    if(entry%2)
    {
        if(test_cloud.points.empty())
            return;

        selectMode = 1;


    }
    else
    {
        selectMode = 0;

    }
}

void test::mousePressEvent(QMouseEvent *event)//鼠标按住
{
    if(Qt::LeftButton == event->button())
    {
        press_Pos = event->pos();
        cout<<press_Pos.x()<<" "<<press_Pos.y()<<endl;
    }
}

void test::mouseReleaseEvent(QMouseEvent *event)
{
    if(Qt::LeftButton == event->button())
    {
        release_Pos = event->pos();
        cout<<release_Pos.x()<<" "<<release_Pos.y()<<endl;
    }
}

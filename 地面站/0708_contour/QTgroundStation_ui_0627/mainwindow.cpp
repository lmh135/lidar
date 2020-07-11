#include "mainwindow.h"
#include "ui_mainwindow.h"

int tool_page = -1;
int tab_page_num = 0;
int tab_current_index = -1;
int color_mode = 0;
bool select_mode = false;

vector<MYCloud> cloud_seq;
vector<Widget*> gl_seq;


int MeasureDisMode = 0;
int which_point = 1;


Contour allcontourlines;
const float ContourElevationUnit=1;//set it for getting contour lines

Filter_Para filter_para;
Registration_Para registration_para;
Fusion_Para fusion_para;
Checkout_Para checkout_para;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    filter_para.filter_index = 0;
    registration_para.registration_index = 0;
    registration_para.cloud1_name = nullptr;
    registration_para.cloud2_name = nullptr;
    registration_para.transformation_matrix = Eigen::Matrix4d::Identity ();
    registration_para.para6.P0.x = 0;
    registration_para.para6.P0.y = 0;
    registration_para.para6.P0.z = 0;
    registration_para.para6.roll = 0;
    registration_para.para6.pitch = 0;
    registration_para.para6.yaw = 0;

    fusion_para.fusion_index = 0;
    fusion_para.trans_para7.P0.x = 0;
    fusion_para.trans_para7.P0.y = 0;
    fusion_para.trans_para7.P0.z = 0;
    fusion_para.trans_para7.u = 0;
    fusion_para.trans_para7.roll = 0;
    fusion_para.trans_para7.pitch = 0;
    fusion_para.trans_para7.yaw = 0;
    fusion_para.world_cloud_name = nullptr;
    fusion_para.pic_cloud_name = nullptr;
    fusion_para.world_point.clear();
    fusion_para.pic_point.clear();

    checkout_para.cloud1_name = nullptr;
    checkout_para.cloud2_name = nullptr;
    checkout_para.trans_para3.dr = 0;
    checkout_para.trans_para3.dp = 0;
    checkout_para.trans_para3.dy = 0;

    tool_page = -1;
    collecting_flag = false;
    tab_current_index = ui->tabWidget->currentIndex();
   // cout<<"page:"<<tab_page<<endl;
    color_mode = 0;
    select_mode = false;
    ui->SaveBtn->setEnabled(false);
    ui->ClearBtn->setEnabled(false);
    ui->SelectBtn->setEnabled(false);
    ui->ColorModeBtn->setEnabled(false);
    ui->Measure_distanceBtn->setEnabled(false);
    ui->RotateBtn->setEnabled(false);
    ui->TranslateBtn->setEnabled(false);
    ui->ZoomBtn->setEnabled(false);
    ui->CloseTabPageBtn->setEnabled(false);

    ui->openSourceDataBtn->setEnabled(true);
    ui->OfflineRecoverBtn->setEnabled(false);
    ui->eulerDetectBtn->setEnabled(false);
    ui->posDetectBtn->setEnabled(false);
    ui->picDetectBtn->setEnabled(false);

    ui->filter_comboBox->setCurrentIndex(0);
    ui->show_filted_cloud_btn->setEnabled(false);
    ui->show_triangulation_btn->setEnabled(false);
    ui->start_filt_btn->setEnabled(false);
    ui->triangute_Btn->setEnabled(false);
//    ui->conture_Btn->setEnabled(false);
    ui->para1_label->setText("参数1:");
    ui->para2_label->setText("参数2:");
    ui->para3_label->setText("参数3:");
    ui->para1_spinbox->setEnabled(false);
    ui->para2_spinbox->setEnabled(false);
    ui->para3_spinbox->setEnabled(false);

    ui->start_registrate_btn->setEnabled(false);
    ui->show_registration_all_btn->setEnabled(false);
    ui->continune_registrate_btn->setEnabled(false);

    ui->Save_collectRecover_Btn->setEnabled(false);

    ui->check_cloud1_combox->setEnabled(false);
    ui->check_cloud2_combox->setEnabled(false);

    //ui->optimizing_start_btn->setEnabled(false);
   // ui->fusion_start_btn->setEnabled(false);


    cloud_seq.clear();
    gl_seq.clear();

    ui->log_view->setModel(&logging_model);
    connect(this, SIGNAL(log_Updated()), this, SLOT(updateLoggingView()));

    connect(this, SIGNAL(start_filt()), &(this->filter), SLOT(start_filter()));
    connect(this, SIGNAL(start_triangulate()), &(this->filter), SLOT(las2triangulation()));


    connect(this, SIGNAL(start_registrate()), &(this->registration), SLOT(start_registration()));
    connect(this, SIGNAL(continue_registrate()), &(this->registration), SLOT(continue_registration()));

    connect(&(this->filter), SIGNAL(filter_ok()), this, SLOT(show_filted_ok()));
    connect(&(this->filter), SIGNAL(triangulate_ok()), this, SLOT(show_triangle_ok()));

    connect(&(this->registration), SIGNAL(registration_ok()), this, SLOT(show_registration_ok()));
    connect(&(this->registration), SIGNAL(continue_step_ok()), this, SLOT(show_registration_ok()));

    connect(&(this->collector.recv_node),SIGNAL(update_Point()),this,SLOT(update_GL()));
    connect(&(this->collector.recv_node),SIGNAL(recv_index_changed()),this,SLOT(change_selected_index()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::update_GL()
{
    vector<Widget*>::iterator item_wid = gl_seq.begin();
    item_wid += tab_current_index;

    (*item_wid)->updateGL();
}

void MainWindow::change_selected_index()
{
    if(tool_page)
        return;

    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if((*item_cloud).selected_index.empty())
        return;

    vector<int>::iterator item_index;

    for(item_index = (*item_cloud).selected_index.begin(); item_index!=(*item_cloud).selected_index.end();)
    {
        (*item_index) = (*item_index)-9600;
        if(((*item_index)<0))
        {

            item_index = (*item_cloud).selected_index.erase(item_index);
        }
        else
            item_index++;

    }

}

void MainWindow::on_action_3_triggered()
{
    exec_flag = 0;
    this->close();
}

void MainWindow::updateLoggingView()
{

    ui->log_view->scrollToBottom();
}

bool MainWindow::loadlasFile(const QString &fileName)
{
    QByteArray fn=(fileName).toLocal8Bit();
    const char* file_name=fn.data();
    cout<<file_name<<endl;

    ifs.open(file_name, ios::in | ios::binary);

    liblas::ReaderFactory f ;
    liblas::Reader reader = f.CreateWithStream(ifs);
    liblas::Header const& header = reader.GetHeader();

    int point_num = header.GetPointRecordsCount();

    cout<<"Number of point records     : "<<point_num<<endl;
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

    MYCloud cloud_in;
    cloud_in.show_index = 0;
   // cout<<"show_index1:"<<cloud_in.show_index<<endl;

    bool isColor = false;

    cloud_in.cloud.points.clear();
    while(reader.ReadNextPoint())
    {
        pcl::PointXYZ p;
        Color rgb;
        int r,g,b;
        r = reader.GetPoint().GetColor().GetRed();
        g = reader.GetPoint().GetColor().GetGreen();
        b = reader.GetPoint().GetColor().GetBlue();
        p.x = reader.GetPoint().GetX();
        p.y = reader.GetPoint().GetY();
        p.z = reader.GetPoint().GetZ();

        if(r|g|b)
            isColor = true;
        rgb.r = r/65536.0f;
        rgb.g = g/65536.0f;
        rgb.b = b/65536.0f;

        cloud_in.cloud.points.push_back(p);
        cloud_in.color.push_back(rgb);
    }
    cout<<"isColor:"<<isColor<<endl;

    cloud_in.isColor = isColor;
    cloud_in.min[0] = header.GetMinX()-0.05;
    cloud_in.min[1] = header.GetMinY()-0.05;
    cloud_in.min[2] = header.GetMinZ()-0.05;
    cloud_in.max[0] = header.GetMaxX()+0.05;
    cloud_in.max[1] = header.GetMaxY()+0.05;
    cloud_in.max[2] = header.GetMaxZ()+0.05;

    cloud_seq.push_back(cloud_in);
    ifs.close();

   // vector<MYCloud>::iterator item_cloud = cloud_seq.end();
   // item_cloud--;
   // cout<<"show_index2:"<<(*item_cloud).show_index<<endl;

    add_cloud_page();

    ui->SaveBtn->setEnabled(true);
    ui->ClearBtn->setEnabled(true);
    ui->SelectBtn->setEnabled(true);
    ui->ColorModeBtn->setEnabled(true);
    ui->Measure_distanceBtn->setEnabled(false);
    ui->RotateBtn->setEnabled(true);
    ui->TranslateBtn->setEnabled(true);
    ui->ZoomBtn->setEnabled(true);
    ui->CloseTabPageBtn->setEnabled(true);


    return 0;
}

void MainWindow::add_cloud_page()
{
    static int ever_num = 0;
    ever_num++;
    tab_page_num++;
    Widget *wid = new Widget(this);

    ui->tabWidget->addTab(wid, "点云图"+QString::number(ever_num));
    ui->tabWidget->setCurrentIndex(tab_page_num-1);

    ui->tabWidget->currentIndex();
    //cout<<"index_tab:"<<index<<endl;
    //cout<<"tab_page_num:"<<tab_page_num<<endl;



    tab_current_index = tab_page_num-1;

    wid->updateGL();
    gl_seq.push_back(wid);
    vector<Widget*>::iterator item = gl_seq.end();
    item--;
    (*item) = wid;

    ui->triangute_Btn->setEnabled(true);

    connect(wid, SIGNAL(release_left_mouse()), this, SLOT(SelectPoint()));


    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    (*item_cloud).mycloud_name = "点云图"+QString::number(ever_num);
    ui->pick_cloud1_combox->addItem((*item_cloud).mycloud_name);
    ui->pick_cloud2_combox->addItem((*item_cloud).mycloud_name);
    ui->pick_world_cloud_combox->addItem((*item_cloud).mycloud_name);
    ui->pick_pic_cloud_combox->addItem((*item_cloud).mycloud_name);
    ui->check_cloud1_combox->addItem((*item_cloud).mycloud_name);
    ui->check_cloud2_combox->addItem((*item_cloud).mycloud_name);
}

inline float sqr(float a)
{
    return a*a;
}
void MainWindow::SelectPoint()
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if((*item_cloud).cloud.points.empty())
        return;

    vector<Widget*>::iterator item_wid = gl_seq.begin();
    item_wid += tab_current_index;

    float press_x,press_y,release_x,release_y;
    press_x = (float)(*item_wid)->pressPos.x();
    press_y = (float)(*item_wid)->pressPos.y();
    release_x = (float)(*item_wid)->releasePos.x();
    release_y = (float)(*item_wid)->releasePos.y();

    float min_x = (*item_cloud).min[0];
    float max_x = (*item_cloud).max[0];

    float min_y = (*item_cloud).min[1];
    float max_y = (*item_cloud).max[1];

    float min_z = (*item_cloud).min[2];
    float max_z = (*item_cloud).max[2];

    float ave_x = (max_x+min_x)/2;
    float ave_y = (max_y+min_y)/2;
    float ave_z = (max_z+min_z)/2;

   // cout<<press_x<<" "<<press_y<<" "<<release_x<<" "<<release_y<<endl;
    Selection_init((*item_cloud).cloud.makeShared());

    int hits = DoOpenGLSelection(press_x,press_y,release_x,release_y,(*item_cloud).cloud.makeShared(),ave_x,ave_y,ave_z);

    (*item_cloud).selected_index.clear();

    (*item_cloud).selected_index = GetSelectedIndex(hits);

    cout<<"共选择了"<<(*item_cloud).selected_index.size()<<"个点云"<<endl;

    Selection_destroy();
/*
    QString temp = (*item_cloud).mycloud_name;
    QByteArray name = temp.toLocal8Bit();
    const char* na = name.data();
    cout<<"my_name:"<<na<<"  selected "<<(*item_cloud).selected_index.size()<<" points"<<endl;
*/

    if(MeasureDisMode)
    {
        double ave_x,ave_y,ave_z;
        float length;
        float last_ave_x;
        int SelectEffect=0;
        for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
        {
            ave_x = ave_x + (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
            ave_y = ave_y + (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
            ave_z = ave_z + (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
        }
        ave_x = ave_x/(*item_cloud).selected_index.size();
        ave_y = ave_y/(*item_cloud).selected_index.size();
        ave_z = ave_z/(*item_cloud).selected_index.size();

        if( ave_x >= min_x && ave_x <= max_x ) SelectEffect=1;
        else SelectEffect=0;
        if(which_point == 1 && SelectEffect==1)
        {
            (*item_cloud).s_ruler_point[0]= ave_x;
            (*item_cloud).s_ruler_point[1]= ave_y;
            (*item_cloud).s_ruler_point[2]= ave_z;
        }
        else if(which_point == 2 && SelectEffect==1)
        {
            (*item_cloud).s_ruler_point[3]= ave_x;
            (*item_cloud).s_ruler_point[4]= ave_y;
            (*item_cloud).s_ruler_point[5]= ave_z;
            length = sqrtf(sqr((*item_cloud).s_ruler_point[0] - (*item_cloud).s_ruler_point[3]) + sqr((*item_cloud).s_ruler_point[1] - (*item_cloud).s_ruler_point[4]) + sqr((*item_cloud).s_ruler_point[2] - (*item_cloud).s_ruler_point[2]));
            //ui->xEdit->setText("distance: "+QString::number(length));
            //cout<<"distance"<<length<<endl;

            std::stringstream logging_model_msg;
            logging_model.insertRows(logging_model.rowCount(),1);
            logging_model_msg << "距离："<<length;

            QVariant new_row(QString(logging_model_msg.str().c_str()));
            logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
            emit log_Updated(); // used to readjust the scrollbar
        }
        if(last_ave_x != ave_x && SelectEffect==1)
        {
            which_point+=1;
            if(which_point>2) which_point=1;
        }
        last_ave_x = ave_x;
       // cout<<s_ruler_point[0]<<" "<<s_ruler_point[1]<<" "<<s_ruler_point[2]<<" "<<s_ruler_point[3]<<" "<<s_ruler_point[4]<<" "<<s_ruler_point[5]<<endl;
    }

   // (*item_wid)->updateGL();
    update_GL();
}

void MainWindow::on_OpenBtn_triggered()
{
    tool_page = -1;

    QString fileName=QFileDialog::getOpenFileName(this,QString("open data"), "", tr("Config Files (*.las)"));
    if(fileName == NULL)
        return;
    loadlasFile(fileName);
}

void MainWindow::on_CloseTabPageBtn_triggered()
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    int current_cloud1_combox_index = ui->pick_cloud1_combox->findText((*item_cloud).mycloud_name);
    int current_cloud2_combox_index = ui->pick_cloud2_combox->findText((*item_cloud).mycloud_name);
    int current_world_cloud_combox_index = ui->pick_world_cloud_combox->findText((*item_cloud).mycloud_name);
    int current_pic_cloud_combox_index = ui->pick_pic_cloud_combox->findText((*item_cloud).mycloud_name);
    int current_check_cloud1_combox_index = ui->check_cloud1_combox->findText((*item_cloud).mycloud_name);
    int current_check_cloud2_combox_index = ui->check_cloud2_combox->findText((*item_cloud).mycloud_name);


    ui->pick_cloud1_combox->removeItem(current_cloud1_combox_index);
    ui->pick_cloud2_combox->removeItem(current_cloud2_combox_index);
    ui->pick_world_cloud_combox->removeItem(current_world_cloud_combox_index);
    ui->pick_pic_cloud_combox->removeItem(current_pic_cloud_combox_index);
    ui->check_cloud1_combox->removeItem(current_check_cloud1_combox_index);
    ui->check_cloud2_combox->removeItem(current_check_cloud2_combox_index);

    cloud_seq.erase(item_cloud);

    vector<Widget*>::iterator item_wid = gl_seq.begin();
    item_wid += tab_current_index;

    gl_seq.erase(item_wid);


    ui->tabWidget->removeTab(tab_current_index);

    tab_page_num--;
    ui->tabWidget->setCurrentIndex(tab_page_num-1);
   // tab_current_index = tab_page_num-1;

    if(tab_page_num == 0)
    {
        ui->SaveBtn->setEnabled(false);
        ui->ClearBtn->setEnabled(false);
        ui->SelectBtn->setEnabled(false);
        ui->ColorModeBtn->setEnabled(false);
        ui->Measure_distanceBtn->setEnabled(false);
        ui->RotateBtn->setEnabled(false);
        ui->TranslateBtn->setEnabled(false);
        ui->ZoomBtn->setEnabled(false);
        ui->CloseTabPageBtn->setEnabled(false);
    }
    if(tool_page == 0)
    {
        collecting_flag = false;
        exec_flag = 0;
    }
}

void draw_fuc()
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if((*item_cloud).show_index==0)
    {
        if((*item_cloud).cloud.points.empty())
            return;

        glPointSize(2.0);
        glBegin( GL_POINTS);

        float min_x = (*item_cloud).min[0];
        float max_x = (*item_cloud).max[0];

        float min_y = (*item_cloud).min[1];
        float max_y = (*item_cloud).max[1];

        float min_z = (*item_cloud).min[2];
        float max_z = (*item_cloud).max[2];
        float boundingBoxZ = max_z-min_z;

        float ave_x = (max_x+min_x)/2;
        float ave_y = (max_y+min_y)/2;
        float ave_z = (max_z+min_z)/2;
       // cout<<"color_mode:"<<color_mode<<endl;
        for(unsigned int i=0;i<(*item_cloud).cloud.points.size();i++)
        {
            if(color_mode)
            {
                float highZ = (*item_cloud).cloud.points[i].z-min_z;


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
                glVertex3f((*item_cloud).cloud.points[i].x-ave_x,(*item_cloud).cloud.points[i].y-ave_y,(*item_cloud).cloud.points[i].z-ave_z);
            }
            else
            {
                if((*item_cloud).isColor)
                    glColor3f((*item_cloud).color[i].r, (*item_cloud).color[i].g, (*item_cloud).color[i].b);
                else
                    glColor3f(0.0f,0.0f,0.0f);
                glVertex3f((*item_cloud).cloud.points[i].x-ave_x,(*item_cloud).cloud.points[i].y-ave_y,(*item_cloud).cloud.points[i].z-ave_z);
            }
        }
        glEnd();

        glPointSize(3.0);
        glBegin( GL_POINTS);
        glColor3f(1.0f,0.0f,0.0f);
        for(unsigned int i=0;i<(*item_cloud).selected_index.size();i++)
            glVertex3f((*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x-ave_x,(*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y-ave_y,(*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z-ave_z);
        glEnd();
/*
        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(min_x-ave_x,min_y-ave_y,min_z-ave_z);
        glVertex3f(max_x-ave_x,min_y-ave_y,min_z-ave_z);
        glVertex3f(max_x-ave_x,max_y-ave_y,min_z-ave_z);
        glVertex3f(min_x-ave_x,max_y-ave_y,min_z-ave_z);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(min_x-ave_x,min_y-ave_y,max_z-ave_z);
        glVertex3f(max_x-ave_x,min_y-ave_y,max_z-ave_z);
        glVertex3f(max_x-ave_x,max_y-ave_y,max_z-ave_z);
        glVertex3f(min_x-ave_x,max_y-ave_y,max_z-ave_z);
        glEnd();
*/
        if(MeasureDisMode && which_point==1)
        {
            glBegin(GL_LINES);
            glLineWidth(3.0);
            glColor3f(0.0f,1.0f,0.0f);
            glVertex3f((*item_cloud).s_ruler_point[0]-ave_x,(*item_cloud).s_ruler_point[1]-ave_y,(*item_cloud).s_ruler_point[2]-ave_z);
            glVertex3f((*item_cloud).s_ruler_point[3]-ave_x,(*item_cloud).s_ruler_point[4]-ave_y,(*item_cloud).s_ruler_point[5]-ave_z);
            glEnd();
        }


    }
    else if((*item_cloud).show_index==1)
    {
        if((*item_cloud).cloud_filted.points.empty())
            return;

        glPointSize(2.0);
        glBegin( GL_POINTS);

        float min_x = (*item_cloud).min[0];
        float max_x = (*item_cloud).max[0];

        float min_y = (*item_cloud).min[1];
        float max_y = (*item_cloud).max[1];

        float min_z = (*item_cloud).min[2];
        float max_z = (*item_cloud).max[2];
        float boundingBoxZ = max_z-min_z;

        float ave_x = (max_x+min_x)/2;
        float ave_y = (max_y+min_y)/2;
        float ave_z = (max_z+min_z)/2;

        for(unsigned int i=0;i<(*item_cloud).cloud_filted.points.size();i++)
        {
            if(color_mode)
            {
                float highZ = (*item_cloud).cloud_filted.points[i].z-min_z;


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
                glVertex3f((*item_cloud).cloud_filted.points[i].x-ave_x,(*item_cloud).cloud_filted.points[i].y-ave_y,(*item_cloud).cloud_filted.points[i].z-ave_z);
            }
            else
            {
                glColor3f(0.0f,0.0f,0.0f);
                glVertex3f((*item_cloud).cloud_filted.points[i].x-ave_x,(*item_cloud).cloud_filted.points[i].y-ave_y,(*item_cloud).cloud_filted.points[i].z-ave_z);
            }
        }
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(min_x-ave_x,min_y-ave_y,min_z-ave_z);
        glVertex3f(max_x-ave_x,min_y-ave_y,min_z-ave_z);
        glVertex3f(max_x-ave_x,max_y-ave_y,min_z-ave_z);
        glVertex3f(min_x-ave_x,max_y-ave_y,min_z-ave_z);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(min_x-ave_x,min_y-ave_y,max_z-ave_z);
        glVertex3f(max_x-ave_x,min_y-ave_y,max_z-ave_z);
        glVertex3f(max_x-ave_x,max_y-ave_y,max_z-ave_z);
        glVertex3f(min_x-ave_x,max_y-ave_y,max_z-ave_z);
        glEnd();

    }
    else if((*item_cloud).show_index==2)
    {
        if((*item_cloud).cloud_triangles.empty())
            return;

        float min_x = (*item_cloud).min[0];
        float max_x = (*item_cloud).max[0];

        float min_y = (*item_cloud).min[1];
        float max_y = (*item_cloud).max[1];

        float min_z = (*item_cloud).min[2];
        float max_z = (*item_cloud).max[2];
        float boundingBoxZ = max_z-min_z;

        float ave_x = (max_x+min_x)/2;
        float ave_y = (max_y+min_y)/2;
        float ave_z = (max_z+min_z)/2;

        glShadeModel(GL_SMOOTH);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);

        glBegin( GL_TRIANGLES );
        for(unsigned int i=0;i<(*item_cloud).cloud_triangles.size();i++)
        {
            float abx=(*item_cloud).cloud_triangles[i].p[1].x-(*item_cloud).cloud_triangles[i].p[0].x;         //计算法线 叉乘
            float aby=(*item_cloud).cloud_triangles[i].p[1].y-(*item_cloud).cloud_triangles[i].p[0].y;
            float abz=(*item_cloud).cloud_triangles[i].p[1].z-(*item_cloud).cloud_triangles[i].p[0].z;

            float bcx=(*item_cloud).cloud_triangles[i].p[2].x-(*item_cloud).cloud_triangles[i].p[0].x;
            float bcy=(*item_cloud).cloud_triangles[i].p[2].y-(*item_cloud).cloud_triangles[i].p[0].y;
            float bcz=(*item_cloud).cloud_triangles[i].p[2].z-(*item_cloud).cloud_triangles[i].p[0].z;
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
            highZ=(*item_cloud).cloud_triangles[i].p[0].z-min_z;
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
            glVertex3f((*item_cloud).cloud_triangles[i].p[0].x-ave_x, (*item_cloud).cloud_triangles[i].p[0].y-ave_y, (*item_cloud).cloud_triangles[i].p[0].z-ave_z);

            highZ=(*item_cloud).cloud_triangles[i].p[1].z-min_z;
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
            glVertex3f((*item_cloud).cloud_triangles[i].p[1].x-ave_x, (*item_cloud).cloud_triangles[i].p[1].y-ave_y, (*item_cloud).cloud_triangles[i].p[1].z-ave_z);

            highZ=(*item_cloud).cloud_triangles[i].p[2].z-min_z;
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
            glVertex3f((*item_cloud).cloud_triangles[i].p[2].x-ave_x, (*item_cloud).cloud_triangles[i].p[2].y-ave_y, (*item_cloud).cloud_triangles[i].p[2].z-ave_z);
       }
       glEnd();
       glDisable(GL_COLOR_MATERIAL);
       glDisable(GL_LIGHTING);
       glDisable(GL_LIGHT0);

//       glBegin(GL_LINE_LOOP);
//       glColor3f(0.0f,1.0f,0.0f);
//       glVertex3f(min_x-ave_x,min_y-ave_y,min_z-ave_z);
//       glVertex3f(max_x-ave_x,min_y-ave_y,min_z-ave_z);
//       glVertex3f(max_x-ave_x,max_y-ave_y,min_z-ave_z);
//       glVertex3f(min_x-ave_x,max_y-ave_y,min_z-ave_z);
//       glEnd();

//       glBegin(GL_LINE_LOOP);
//       glColor3f(0.0f,1.0f,0.0f);
//       glVertex3f(min_x-ave_x,min_y-ave_y,max_z-ave_z);
//       glVertex3f(max_x-ave_x,min_y-ave_y,max_z-ave_z);
//       glVertex3f(max_x-ave_x,max_y-ave_y,max_z-ave_z);
//       glVertex3f(min_x-ave_x,max_y-ave_y,max_z-ave_z);
//       glEnd();

    }
    else if((*item_cloud).show_index==3)
    {
        if((*item_cloud).cloud_filted_triangles.empty())
            return;

        float min_x = (*item_cloud).min[0];
        float max_x = (*item_cloud).max[0];

        float min_y = (*item_cloud).min[1];
        float max_y = (*item_cloud).max[1];

        float min_z = (*item_cloud).min[2];
        float max_z = (*item_cloud).max[2];
        float boundingBoxZ = max_z-min_z;

        float ave_x = (max_x+min_x)/2;
        float ave_y = (max_y+min_y)/2;
        float ave_z = (max_z+min_z)/2;

        glShadeModel(GL_SMOOTH);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
        glEnable(GL_LIGHT0);
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);

        glBegin( GL_TRIANGLES );
        for(unsigned int i=0;i<(*item_cloud).cloud_filted_triangles.size();i++)
        {
            float abx=(*item_cloud).cloud_filted_triangles[i].p[1].x-(*item_cloud).cloud_filted_triangles[i].p[0].x;         //计算法线 叉乘
            float aby=(*item_cloud).cloud_filted_triangles[i].p[1].y-(*item_cloud).cloud_filted_triangles[i].p[0].y;
            float abz=(*item_cloud).cloud_filted_triangles[i].p[1].z-(*item_cloud).cloud_filted_triangles[i].p[0].z;

            float bcx=(*item_cloud).cloud_filted_triangles[i].p[2].x-(*item_cloud).cloud_filted_triangles[i].p[0].x;
            float bcy=(*item_cloud).cloud_filted_triangles[i].p[2].y-(*item_cloud).cloud_filted_triangles[i].p[0].y;
            float bcz=(*item_cloud).cloud_filted_triangles[i].p[2].z-(*item_cloud).cloud_filted_triangles[i].p[0].z;
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
            highZ=(*item_cloud).cloud_filted_triangles[i].p[0].z-min_z;
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
            glVertex3f((*item_cloud).cloud_filted_triangles[i].p[0].x-ave_x, (*item_cloud).cloud_filted_triangles[i].p[0].y-ave_y, (*item_cloud).cloud_filted_triangles[i].p[0].z-ave_z);

            highZ=(*item_cloud).cloud_filted_triangles[i].p[1].z-min_z;
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
            glVertex3f((*item_cloud).cloud_filted_triangles[i].p[1].x-ave_x, (*item_cloud).cloud_filted_triangles[i].p[1].y-ave_y, (*item_cloud).cloud_filted_triangles[i].p[1].z-ave_z);

            highZ=(*item_cloud).cloud_filted_triangles[i].p[2].z-min_z;
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
            glVertex3f((*item_cloud).cloud_filted_triangles[i].p[2].x-ave_x, (*item_cloud).cloud_filted_triangles[i].p[2].y-ave_y, (*item_cloud).cloud_filted_triangles[i].p[2].z-ave_z);
       }
       glEnd();
       glDisable(GL_COLOR_MATERIAL);
       glDisable(GL_LIGHTING);
       glDisable(GL_LIGHT0);

       glBegin(GL_LINE_LOOP);
       glColor3f(0.0f,1.0f,0.0f);
       glVertex3f(min_x-ave_x,min_y-ave_y,min_z-ave_z);
       glVertex3f(max_x-ave_x,min_y-ave_y,min_z-ave_z);
       glVertex3f(max_x-ave_x,max_y-ave_y,min_z-ave_z);
       glVertex3f(min_x-ave_x,max_y-ave_y,min_z-ave_z);
       glEnd();

       glBegin(GL_LINE_LOOP);
       glColor3f(0.0f,1.0f,0.0f);
       glVertex3f(min_x-ave_x,min_y-ave_y,max_z-ave_z);
       glVertex3f(max_x-ave_x,min_y-ave_y,max_z-ave_z);
       glVertex3f(max_x-ave_x,max_y-ave_y,max_z-ave_z);
       glVertex3f(min_x-ave_x,max_y-ave_y,max_z-ave_z);
       glEnd();
    }
    else if((*item_cloud).show_index==4)
    {
        if((*item_cloud).cloud_registrated1.points.empty())
            return;
        if((*item_cloud).cloud_registrated2.points.empty())
            return;

        glPointSize(2.0);
        glBegin( GL_POINTS);

        float min_x = (*item_cloud).min[0];
        float max_x = (*item_cloud).max[0];

        float min_y = (*item_cloud).min[1];
        float max_y = (*item_cloud).max[1];

        float min_z = (*item_cloud).min[2];
        float max_z = (*item_cloud).max[2];
        float boundingBoxZ = max_z-min_z;

        float ave_x = (max_x+min_x)/2;
        float ave_y = (max_y+min_y)/2;
        float ave_z = (max_z+min_z)/2;

        for(unsigned int i=0;i<(*item_cloud).cloud_registrated1.points.size();i++)
        {
            if(color_mode)
            {
                float highZ = (*item_cloud).cloud_registrated1.points[i].z-min_z;


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
                glVertex3f((*item_cloud).cloud_registrated1.points[i].x-ave_x,(*item_cloud).cloud_registrated1.points[i].y-ave_y,(*item_cloud).cloud_registrated1.points[i].z-ave_z);
            }
            else
            {
                glColor3f(0.0f,0.0f,0.0f);
                glVertex3f((*item_cloud).cloud_registrated1.points[i].x-ave_x,(*item_cloud).cloud_registrated1.points[i].y-ave_y,(*item_cloud).cloud_registrated1.points[i].z-ave_z);
            }
        }
        glEnd();

        glBegin( GL_POINTS);
        for(unsigned int i=0;i<(*item_cloud).cloud_registrated2.points.size();i++)
        {
            glColor3f(0.0f,0.0f,1.0f);
            glVertex3f((*item_cloud).cloud_registrated2.points[i].x-ave_x,(*item_cloud).cloud_registrated2.points[i].y-ave_y,(*item_cloud).cloud_registrated2.points[i].z-ave_z);
        }
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(min_x-ave_x,min_y-ave_y,min_z-ave_z);
        glVertex3f(max_x-ave_x,min_y-ave_y,min_z-ave_z);
        glVertex3f(max_x-ave_x,max_y-ave_y,min_z-ave_z);
        glVertex3f(min_x-ave_x,max_y-ave_y,min_z-ave_z);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(min_x-ave_x,min_y-ave_y,max_z-ave_z);
        glVertex3f(max_x-ave_x,min_y-ave_y,max_z-ave_z);
        glVertex3f(max_x-ave_x,max_y-ave_y,max_z-ave_z);
        glVertex3f(min_x-ave_x,max_y-ave_y,max_z-ave_z);
        glEnd();
    }
}

void MainWindow::on_SaveBtn_triggered()
{
    QString filename = QFileDialog::getSaveFileName(this, QString("save data"), "", tr("Config Files (*.las)"));

    if(filename==NULL)
        return;
    QString txtfilename = filename + ".txt";
    QString lasfilename = filename + ".las";

    QByteArray fn_txt = txtfilename.toLocal8Bit();
    const char* txtfile_name = fn_txt.data();
    QByteArray fn_las = lasfilename.toLocal8Bit();
    const char* lasfile_name = fn_las.data();

    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if( (*item_cloud).show_index==0)
    {
//        if(!collecting_flag)
//        {
        if((*item_cloud).isColor == false)
        {
            FILE* p1=NULL;
            p1 = fopen(txtfile_name,"a+");
            for(unsigned int i=0; i<(*item_cloud).cloud.points.size(); i++)
            {
                fprintf(p1,"%lf %lf %lf \n",(*item_cloud).cloud.points[i].x,(*item_cloud).cloud.points[i].y,(*item_cloud).cloud.points[i].z);
            }
            fclose(p1);

            char str[512];
            sprintf(str,"gnome-terminal -x bash -c \"txt2las %s -o %s; exec bash\"",txtfile_name,lasfile_name);


            QProcess *pro=new QProcess();
            pro->start(str);
        }
        else
        {
            FILE* p1=NULL;
            p1 = fopen(txtfile_name,"a+");
            for(unsigned int i=0; i<(*item_cloud).cloud.points.size(); i++)
            {
                int r,g,b;
                r = (int)((*item_cloud).color[i].r * 65536);
                g = (int)((*item_cloud).color[i].g * 65536);
                b = (int)((*item_cloud).color[i].b * 65536);
                fprintf(p1,"%lf %lf %lf %d %d %d\n",(*item_cloud).cloud.points[i].x,(*item_cloud).cloud.points[i].y,(*item_cloud).cloud.points[i].z,r,g,b);
            }
            fclose(p1);

            char str[512];
            sprintf(str,"gnome-terminal -x bash -c \"txt2las %s --parse xyzRGB -o %s; exec bash\"",txtfile_name,lasfile_name);


            QProcess *pro=new QProcess();
            pro->start(str);

        }
//        }
//        else
//        {
//            if(points_recv.points.empty())
//            {
//                QWidget *temp = new QWidget;
//                QMessageBox::information(temp,"错误","采集点云为空!");
//                delete temp;
//                return;
//            }
//            else
//            {
//                FILE* p1=NULL;
//                p1 = fopen(txtfile_name,"a+");
//                for(unsigned int i=0; i<points_recv.points.size(); i++)
//                {
//                    fprintf(p1,"%lf %lf %lf \n",points_recv.points[i].x,points_recv.points[i].y,points_recv.points[i].z);
//                }
//                fclose(p1);

//                char str[512];
//                sprintf(str,"gnome-terminal -x bash -c \"txt2las %s -o %s; exec bash\"",txtfile_name,lasfile_name);


//                QProcess *pro=new QProcess();
//                pro->start(str);
//            }
//        }
    }

    else if( (*item_cloud).show_index==1 ||(*item_cloud).show_index==2||(*item_cloud).show_index==3)
    {
            vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
            item_cloud += tab_current_index;

            FILE* p1=NULL;
            p1 = fopen(txtfile_name,"a+");
            for(unsigned int i=0; i<(*item_cloud).cloud_filted.points.size(); i++)
            {
                fprintf(p1,"%lf %lf %lf \n",(*item_cloud).cloud_filted.points[i].x,(*item_cloud).cloud_filted.points[i].y,(*item_cloud).cloud_filted.points[i].z);
            }
            fclose(p1);

            char str[512];
            sprintf(str,"gnome-terminal -x bash -c \"txt2las %s -o %s; exec bash\"",txtfile_name,lasfile_name);


            QProcess *pro=new QProcess();
            pro->start(str);
    }

}

void MainWindow::on_Save_collectRecover_Btn_clicked()
{

    QString filename = QFileDialog::getSaveFileName(this, QString("save data"), "", tr("Config Files (*.las)"));

    if(filename==NULL)
        return;
    QString txtfilename = filename + ".txt";
    QString lasfilename = filename + ".las";

    QByteArray fn_txt = txtfilename.toLocal8Bit();
    const char* txtfile_name = fn_txt.data();
    QByteArray fn_las = lasfilename.toLocal8Bit();
    const char* lasfile_name = fn_las.data();

    if(points_recv.points.empty())
    {
        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"错误","采集点云为空!");
        delete temp;
        return;
    }
    else
    {
        FILE* p1=NULL;
        p1 = fopen(txtfile_name,"a+");
        for(unsigned int i=0; i<points_recv.points.size(); i++)
        {
            fprintf(p1,"%lf %lf %lf \n",points_recv.points[i].x,points_recv.points[i].y,points_recv.points[i].z);
        }
        fclose(p1);

        char str[512];
        sprintf(str,"gnome-terminal -x bash -c \"txt2las %s -o %s; exec bash\"",txtfile_name,lasfile_name);


        QProcess *pro=new QProcess();
        pro->start(str);
    }
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
    if(collecting_flag)
    {
        ui->tabWidget->setCurrentIndex(tab_current_index);
        //cout<<"hahahaindex_tab:"<<tab_current_index<<endl;
        return;
    }
    tab_current_index = index;
    //cout<<"tab_current_index:"<<tab_current_index<<endl;

}

void MainWindow::on_ColorModeBtn_triggered()
{
    color_mode++;
    color_mode = color_mode%2;

//    vector<Widget*>::iterator item = gl_seq.begin();
//    item += tab_current_index;

//    (*item)->updateGL();
    update_GL();
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(!select_mode)
        return;

    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if(Qt::Key_Delete == event->key())//删除选中的点云
    {
        if((*item_cloud).selected_index.empty())
            return;


        for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
        {
            (*item_cloud).cloud.points.erase((*item_cloud).cloud.points.begin()+((*item_cloud).selected_index[i]-i));
            (*item_cloud).color.erase((*item_cloud).color.begin()+((*item_cloud).selected_index[i]-i));
        }


        (*item_cloud).selected_index.clear();
    }
    if(Qt::Key_P == event->key())//删除片选外的点云
    {
        if((*item_cloud).selected_index.empty())
            return;
        vector<Point> p_selected;
        vector<Color> color_selected;
        for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
        {
            Point p;
            p.x = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
            p.y = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
            p.z = (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
            p_selected.push_back(p);

            Color c;
            c.r = (*item_cloud).color[(*item_cloud).selected_index[i]].r;
            c.g = (*item_cloud).color[(*item_cloud).selected_index[i]].g;
            c.b = (*item_cloud).color[(*item_cloud).selected_index[i]].b;
            color_selected.push_back(c);
        }
        (*item_cloud).cloud.points.clear();
        (*item_cloud).color.clear();
        for(unsigned int i=0; i<p_selected.size(); i++)
        {
            pcl::PointXYZ p;
            p.x = p_selected[i].x;
            p.y = p_selected[i].y;
            p.z = p_selected[i].z;
            (*item_cloud).cloud.points.push_back(p);

            Color c = color_selected[i];
            (*item_cloud).color.push_back(c);

        }
        (*item_cloud).selected_index.clear();
    }
    if(Qt::Key_S == event->key())//显示被选中第一个点的坐标
    {
        cout<<'s'<<endl;
        if((*item_cloud).selected_index.empty())
            return;

        std::stringstream logging_model_msg;
        logging_model.insertRows(logging_model.rowCount(),1);
        logging_model_msg << "坐标:("<<(*item_cloud).cloud.points[(*item_cloud).selected_index[0]].x<<", "<<(*item_cloud).cloud.points[(*item_cloud).selected_index[0]].y<< \
                          ", "<<(*item_cloud).cloud.points[(*item_cloud).selected_index[0]].z<<")";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar





    }
    if(Qt::Key_C == event->key())//显示被选点的平均坐标
    {
        if((*item_cloud).selected_index.empty())
            return;

        double ave_x,ave_y,ave_z;
        for(unsigned int i=0; i<(*item_cloud).selected_index.size(); i++)
        {
            ave_x = ave_x + (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].x;
            ave_y = ave_y + (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].y;
            ave_z = ave_z + (*item_cloud).cloud.points[(*item_cloud).selected_index[i]].z;
        }
        ave_x = ave_x/(*item_cloud).selected_index.size();
        ave_y = ave_y/(*item_cloud).selected_index.size();
        ave_z = ave_z/(*item_cloud).selected_index.size();

        std::stringstream logging_model_msg;
        logging_model.insertRows(logging_model.rowCount(),1);
        logging_model_msg << "平均坐标:("<<ave_x<<", "<<ave_y<<", "<<ave_z<<")";

        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar
    }
    update_GL();
}

void MainWindow::on_SelectBtn_triggered()
{
    select_mode = !select_mode;
    if(select_mode)
    {
        ui->SaveBtn->setEnabled(false);
        ui->ClearBtn->setEnabled(false);
       // ui->SelectBtn->setEnabled(false);
        ui->ColorModeBtn->setEnabled(false);
        ui->Measure_distanceBtn->setEnabled(true);
        ui->RotateBtn->setEnabled(false);
        ui->TranslateBtn->setEnabled(false);
        ui->ZoomBtn->setEnabled(false);
        ui->CloseTabPageBtn->setEnabled(false);
        std::stringstream logging_model_msg;
        logging_model.insertRows(logging_model.rowCount(),1);
        logging_model_msg << "已进入选择模式,键盘操作如下："<<endl<<"Delete: 删除已选中的点云"<<endl<<"P: 保留已选中的点云"<<endl<< \
                             "S: 显示被选点云坐标"<<endl<<"C: 显示被选点云平均坐标";

        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar
    }
    else
    {
        ui->SaveBtn->setEnabled(true);
        ui->ClearBtn->setEnabled(true);
       // ui->SelectBtn->setEnabled(false);
        ui->ColorModeBtn->setEnabled(true);
        ui->Measure_distanceBtn->setEnabled(false);
        ui->RotateBtn->setEnabled(true);
        ui->TranslateBtn->setEnabled(true);
        ui->ZoomBtn->setEnabled(true);
        ui->CloseTabPageBtn->setEnabled(true);

        std::stringstream logging_model_msg;
        logging_model.insertRows(logging_model.rowCount(),1);
        logging_model_msg << "已退出选择模式";

        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar
    }
}

void MainWindow::on_Measure_distanceBtn_triggered()
{
        MeasureDisMode++;
        MeasureDisMode = MeasureDisMode%2;
        if(MeasureDisMode==1)
        {
            std::stringstream logging_model_msg;
            logging_model.insertRows(logging_model.rowCount(),1);
            logging_model_msg << "已进入测距模式,请选择两个点";
            QVariant new_row(QString(logging_model_msg.str().c_str()));
            logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
            emit log_Updated(); // used to readjust the scrollbar
        }
        else
        {
            std::stringstream logging_model_msg;
            logging_model.insertRows(logging_model.rowCount(),1);
            logging_model_msg << "已退出测距模式";
            QVariant new_row(QString(logging_model_msg.str().c_str()));
            logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
            emit log_Updated(); // used to readjust the scrollbar
        }
}

void MainWindow::on_openSourceDataBtn_clicked()
{
    tool_page = 0;
    collecting_flag = true;
    ui->OpenBtn->setEnabled(false);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);

    if(tab_page_num == 0)
    {
        MYCloud temp;
        temp.show_index = 0;
        temp.selected_index.clear();
        cloud_seq.push_back(temp);
        add_cloud_page();
    }
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    (*item_cloud).cloud.points.clear();


    QString fileName = QFileDialog::getOpenFileName(this,QString("open data"));

    if(fileName == NULL)
    {
        logging_model_msg << "文件为空!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

        return;
    }
    else
    {
        this->collector.fileName = fileName;
        ui->OfflineRecoverBtn->setEnabled(true);

        logging_model_msg << "文件载入成功!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

        return;
    }



}

void MainWindow::on_connect_Btn_clicked()
{
    tool_page = 0;
    collecting_flag = true;

    ui->OpenBtn->setEnabled(false);
    if(tab_page_num == 0)
    {
        MYCloud temp;
        temp.selected_index.clear();
        cloud_seq.push_back(temp);
        add_cloud_page();
    }
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    (*item_cloud).cloud.points.clear();


    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"cd ; exec bash\"");
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "建立连接中!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}

void MainWindow::on_OfflineRecoverBtn_clicked()
{
    tool_page = 0;
    collecting_flag = true;
    QByteArray fn = this->collector.fileName.toLocal8Bit();
    const char* file_name=fn.data();
    cout<<file_name<<endl;


    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"roslaunch lidar_station lidar_offline.launch file-name:=%s; exec bash\"",file_name);
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    ui->eulerDetectBtn->setEnabled(true);
    ui->posDetectBtn->setEnabled(true);
    ui->picDetectBtn->setEnabled(true);




    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "离线恢复开始!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar

}

void MainWindow::on_OnlineRecoverBtn_clicked()
{
    tool_page = 0;
    collecting_flag = true;

    ui->OpenBtn->setEnabled(false);

    if(tab_page_num == 0)
    {
        MYCloud temp;
        temp.selected_index.clear();
        cloud_seq.push_back(temp);
        add_cloud_page();
    }
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    (*item_cloud).cloud.points.clear();


    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"roslaunch lidar_station lidar_online.launch ; exec bash\"");
   // system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    ui->eulerDetectBtn->setEnabled(true);
    ui->posDetectBtn->setEnabled(true);
    ui->picDetectBtn->setEnabled(true);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "在线采集开始!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar

}

void MainWindow::on_posDetectBtn_clicked()
{
    tool_page = 0;
    collecting_flag = true;
    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"rqt_plot /gps_packet/ned_xyz\"");
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "姿态检测中!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar

}

void MainWindow::on_eulerDetectBtn_clicked()
{
    tool_page = 0;
    collecting_flag = true;
    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"rqt_plot /imu_packet/imu_euler\"");
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "位置检测中!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar

}

void MainWindow::on_point_show_Btn_clicked()
{
    tool_page = 0;
    collecting_flag = true;

    static bool recv_flag = false;


    if(tab_page_num == 0)
    {
        MYCloud temp;
        cloud_seq.push_back(temp);
        add_cloud_page();
    }
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    (*item_cloud).cloud.points.clear();


    recv_flag = !recv_flag;
    if(recv_flag == true)
    {
        points_recv.points.clear();
        exec_flag = 1;
        ui->OpenBtn->setEnabled(false);
        ui->point_show_Btn->setText("暂停显示");
        this->collector.recv_node.init();
        std::stringstream logging_model_msg;
        logging_model.insertRows(logging_model.rowCount(),1);
        logging_model_msg << "正在显示!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

    }
    else
    {
        exec_flag = 0;
        collecting_flag = false;
        ui->OpenBtn->setEnabled(true);

        ui->Save_collectRecover_Btn->setEnabled(true);


        ui->point_show_Btn->setText("点云显示");
        std::stringstream logging_model_msg;
        logging_model.insertRows(logging_model.rowCount(),1);
        logging_model_msg << "已停止!请保存已采集的点云!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

    }

    ui->SaveBtn->setEnabled(true);
    ui->ClearBtn->setEnabled(true);
    ui->SelectBtn->setEnabled(true);
    ui->ColorModeBtn->setEnabled(true);
    ui->Measure_distanceBtn->setEnabled(false);
    ui->RotateBtn->setEnabled(true);
    ui->TranslateBtn->setEnabled(true);
    ui->ZoomBtn->setEnabled(true);
    ui->CloseTabPageBtn->setEnabled(true);



}

void MainWindow::on_ClearBtn_triggered()
{
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    (*item_cloud).cloud.points.clear();
    (*item_cloud).selected_index.clear();

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "清除点云!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}

void MainWindow::on_toolBox_currentChanged(int index)
{
    cout<<"tool_page:"<<index<<endl;
    if(exec_flag)
        ui->toolBox->setCurrentIndex(0);
    else
        tool_page = index;



    if(tool_page==1)
    {
        filter_para.filter_index = 0;
        ui->filter_comboBox->setCurrentIndex(0);
        ui->start_filt_btn->setEnabled(false);
        ui->show_filted_cloud_btn->setEnabled(false);
        ui->show_triangulation_btn->setEnabled(false);
//        ui->conture_Btn->setEnabled(false);
        ui->para1_label->setText("参数1:");
        ui->para2_label->setText("参数2:");
        ui->para3_label->setText("参数3:");
        ui->para1_spinbox->setEnabled(false);
        ui->para2_spinbox->setEnabled(false);
        ui->para3_spinbox->setEnabled(false);

    }


}

void MainWindow::on_filter_comboBox_currentIndexChanged(int index)
{
    if(index==0)
        return;
    switch(index)
    {
    case 1:
        filter_para.filter_index = 1;
        ui->para1_label->setText("K近邻数:");
        ui->para2_label->setText("参数2:");
        ui->para3_label->setText("参数3:");
        ui->para1_spinbox->setEnabled(true);
        ui->para2_spinbox->setEnabled(false);
        ui->para3_spinbox->setEnabled(false);
        ui->para1_spinbox->setRange(10,100);
        ui->para1_spinbox->setValue(50);
        break;
    case 2:
        filter_para.filter_index = 2;
        ui->para1_label->setText("K近邻数:");
        ui->para2_label->setText("法向权重:");
        ui->para3_label->setText("参数3:");
        ui->para1_spinbox->setEnabled(true);
        ui->para2_spinbox->setEnabled(true);
        ui->para3_spinbox->setEnabled(false);
        ui->para1_spinbox->setRange(10,100);
        ui->para2_spinbox->setRange(0,1);
        ui->para1_spinbox->setValue(50);
        ui->para2_spinbox->setValue(0.8);
        break;
    case 3:
        filter_para.filter_index = 3;
        ui->para1_label->setText("最大厚度:");
        ui->para2_label->setText("最小厚度:");
        ui->para3_label->setText("允许厚度:");
        ui->para1_spinbox->setEnabled(true);
        ui->para2_spinbox->setEnabled(true);
        ui->para3_spinbox->setEnabled(true);
        ui->para1_spinbox->setRange(1,2);
        ui->para2_spinbox->setRange(0.4,1);
        ui->para2_spinbox->setRange(0.1,0.4);
        ui->para1_spinbox->setValue(1.4);
        ui->para2_spinbox->setValue(0.4);
        ui->para3_spinbox->setValue(0.3);
        break;
    case 4:
        filter_para.filter_index = 4;
        ui->para1_label->setText("栅格大小:");
        ui->para2_label->setText("参数2:");
        ui->para3_label->setText("参数3:");
        ui->para1_spinbox->setEnabled(true);
        ui->para2_spinbox->setEnabled(false);
        ui->para3_spinbox->setEnabled(false);
        ui->para1_spinbox->setRange(0.1,1);
        ui->para1_spinbox->setValue(0.3);
        break;
    case 5:
        filter_para.filter_index = 5;
        ui->para1_label->setText("K近邻数:");
        ui->para2_label->setText("参数2:");
        ui->para3_label->setText("参数3:");
        ui->para1_spinbox->setEnabled(true);
        ui->para2_spinbox->setEnabled(false);
        ui->para3_spinbox->setEnabled(false);
        ui->para1_spinbox->setRange(10,100);
        ui->para1_spinbox->setValue(50);
        break;
    case 6:
        filter_para.filter_index = 6;
        ui->para1_label->setText("栅格大小:");
        ui->para2_label->setText("参数2:");
        ui->para3_label->setText("参数3:");
        ui->para1_spinbox->setEnabled(true);
        ui->para2_spinbox->setEnabled(false);
        ui->para3_spinbox->setEnabled(false);
        ui->para1_spinbox->setRange(0.1,1);
        ui->para1_spinbox->setValue(0.3);
        break;
    default:
        break;

    }
}

void MainWindow::on_para1_spinbox_valueChanged(double arg1)
{
   // cout<<"arg:"<<arg1<<endl;
    int filter_index = filter_para.filter_index;
    if(filter_index == 0)
        return;
    switch(filter_index)
    {
    case 1:
        filter_para.KNear = arg1;
        ui->start_filt_btn->setEnabled(true);
        break;
    case 2:
        filter_para.KNear = arg1;
        ui->start_filt_btn->setEnabled(true);
        break;
    case 3:
        filter_para.max_thick = arg1;
        ui->start_filt_btn->setEnabled(true);
        break;
    case 4:
        filter_para.GridSize = arg1;
        ui->start_filt_btn->setEnabled(true);
        break;
    case 5:
        filter_para.KNear = arg1;
        ui->start_filt_btn->setEnabled(true);
        break;
    case 6:
        filter_para.GridSize = arg1;
        ui->start_filt_btn->setEnabled(true);
        break;
    default:
        break;
    }
}

void MainWindow::on_para2_spinbox_valueChanged(double arg1)
{
    int filter_index = filter_para.filter_index;
    if(filter_index == 0)
        return;
    switch(filter_index)
    {
    case 1:
        break;
    case 2:
        filter_para.NormalDirectionAdjust = arg1;
        break;
    case 3:
        filter_para.min_thick = arg1;
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    default:
        break;
    }
}

void MainWindow::on_para3_spinbox_valueChanged(double arg1)
{
    int filter_index = filter_para.filter_index;
    if(filter_index == 0)
        return;
    switch(filter_index)
    {
    case 1:
        break;
    case 2:
        break;
    case 3:
        filter_para.selected_thick = arg1;
        break;
    case 4:
        break;
    case 5:
        break;
    case 6:
        break;
    default:
        break;
    }
}

void MainWindow::on_start_filt_btn_clicked()
{
    emit start_filt();
}

void MainWindow::show_filted_ok()
{
    ui->show_filted_cloud_btn->setEnabled(true);

}

void MainWindow::show_triangle_ok()
{
    ui->show_triangulation_btn->setEnabled(true);
}

void MainWindow::show_registration_ok()
{
    ui->show_registration_all_btn->setEnabled(true);
   // ui->continune_registrate_btn->setEnabled(true);
    update_GL();
}

void MainWindow::on_show_filted_cloud_btn_clicked()
{
    static bool entry = false;
    entry = !entry;
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if(entry)
    {
        ui->show_filted_cloud_btn->setText("查看原始点云");

        (*item_cloud).show_index = 1;
        ui->Measure_distanceBtn->setEnabled(false);
       // cout<<"hahah:12"<<endl;

    }
    else
    {
        ui->show_filted_cloud_btn->setText("查看滤波点云");
        (*item_cloud).show_index = 0;
        ui->Measure_distanceBtn->setEnabled(true);
       // cout<<"hahah:111111"<<endl;
    }
    update_GL();
}

void MainWindow::on_show_triangulation_btn_clicked()
{
    static bool entry = false;
    entry = !entry;
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if(entry)
    {
        ui->show_triangulation_btn->setText("查看原始点云");
        if((*item_cloud).show_index==0)
            (*item_cloud).show_index = 2;
        else if((*item_cloud).show_index==1)
            (*item_cloud).show_index = 3;
        ui->Measure_distanceBtn->setEnabled(false);
        ui->SelectBtn->setEnabled(false);
       // cout<<"hahah:12"<<endl;

    }
    else
    {
        ui->show_triangulation_btn->setText("查看三角化");
        if((*item_cloud).show_index==2)
            (*item_cloud).show_index = 0;
        else if((*item_cloud).show_index==3)
            (*item_cloud).show_index = 1;
        ui->SelectBtn->setEnabled(true);
        //ui->Measure_distanceBtn->setEnabled(true);
       // cout<<"hahah:111111"<<endl;
    }
    update_GL();
}

void MainWindow::on_triangute_Btn_clicked()
{
    emit start_triangulate();
}

void MainWindow::on_pick_cloud1_combox_currentIndexChanged(const QString &arg1)
{
    registration_para.cloud1_name = arg1;
}

void MainWindow::on_pick_cloud2_combox_currentIndexChanged(const QString &arg1)
{
    registration_para.cloud2_name = arg1;
}

void MainWindow::on_pick_world_cloud_combox_currentIndexChanged(const QString &arg1)
{
    fusion_para.world_cloud_name = arg1;

    for(vector<MYCloud>::iterator item_temp = cloud_seq.begin(); item_temp!=cloud_seq.end(); item_temp++)
    {
        if(fusion_para.world_cloud_name == (*item_temp).mycloud_name)
        {
            if((*item_temp).cloud.points.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","世界点云为空!");
                delete temp;
                return;
            }
            if((*item_temp).selected_index.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","世界点云还未选择公共区域!");
                delete temp;
                return;
            }

            pcl::PointCloud<pcl::PointXYZ> cloud_temp;
            for(unsigned int i=0; i<(*item_temp).selected_index.size(); i++)
            {
                pcl::PointXYZ p;
                p = (*item_temp).cloud.points[(*item_temp).selected_index[i]];
                cloud_temp.points.push_back(p);
            }

            fusion.init_world_cloud_selected(cloud_temp);
            fusion.init_world_cloud_whole((*item_temp).cloud);
        }

    }
}

void MainWindow::on_pick_pic_cloud_combox_currentIndexChanged(const QString &arg1)
{
    fusion_para.pic_cloud_name = arg1;

    for(vector<MYCloud>::iterator item_temp = cloud_seq.begin(); item_temp!=cloud_seq.end(); item_temp++)
    {
        if(fusion_para.pic_cloud_name == (*item_temp).mycloud_name)
        {
            if((*item_temp).cloud.points.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","图像点云为空!");
                delete temp;
                return;
            }
            if((*item_temp).selected_index.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","图像点云还未选择公共区域!");
                delete temp;
                return;
            }

            pcl::PointCloud<pcl::PointXYZ> cloud_temp;
            for(unsigned int i=0; i<(*item_temp).selected_index.size(); i++)
            {
                pcl::PointXYZ p;
                p = (*item_temp).cloud.points[(*item_temp).selected_index[i]];
                cloud_temp.points.push_back(p);
            }

            fusion.init_pic_cloud_selected(cloud_temp);
            fusion.init_pic_cloud_whole((*item_temp).cloud, (*item_temp).color);
        }

    }
}

void MainWindow::on_check_cloud1_combox_currentIndexChanged(const QString &arg1)
{
    checkout_para.cloud1_name = arg1;

    for(vector<MYCloud>::iterator item_temp = cloud_seq.begin(); item_temp!=cloud_seq.end(); item_temp++)
    {
        if(checkout_para.cloud1_name == (*item_temp).mycloud_name)
        {
            if((*item_temp).cloud.points.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","点云图为空!");
                delete temp;
                return;
            }
            if((*item_temp).selected_index.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","点云图还未选择公共区域!");
                delete temp;
                return;
            }

            checkout_rpy.ori_cloud1.clear();

            QByteArray fn= checkout_rpy.binary_file_cloud1.toLocal8Bit();
            const char* file_name=fn.data();
            cout<<file_name<<endl;

            FILE* binary_file = fopen(fn,"rb+");

            Ori_data tdata;
            for(unsigned int i=0; i<(*item_temp).selected_index.size(); i++)
            {
                int id_num = (*item_temp).selected_index[i];
                fseek(binary_file,(id_num)*sizeof(Ori_data),SEEK_SET);  //待验证
                int num_to_read = fread(&tdata,sizeof(Ori_data),1,binary_file);

                if(num_to_read != 1)
                {
                    QWidget *temp = new QWidget;
                    QMessageBox::information(temp,"错误","原始数据1读取失败！");
                    delete temp;
                    checkout_rpy.ori_cloud1.clear();
                    return;
                }

                checkout_rpy.ori_cloud1.push_back(tdata);
            }
            fclose(binary_file);

        }

    }
}

void MainWindow::on_check_cloud2_combox_currentIndexChanged(const QString &arg1)
{
    checkout_para.cloud2_name = arg1;

    for(vector<MYCloud>::iterator item_temp = cloud_seq.begin(); item_temp!=cloud_seq.end(); item_temp++)
    {
        if(checkout_para.cloud2_name == (*item_temp).mycloud_name)
        {
            if((*item_temp).cloud.points.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","点云图为空!");
                delete temp;
                return;
            }
            if((*item_temp).selected_index.empty())
            {
                QWidget *temp = new QWidget;
                QMessageBox::information(temp,"错误","点云图还未选择公共区域!");
                delete temp;
                return;
            }

            checkout_rpy.ori_cloud2.clear();

            QByteArray fn= checkout_rpy.binary_file_cloud2.toLocal8Bit();
            const char* file_name=fn.data();
            cout<<file_name<<endl;

            FILE* binary_file = fopen(fn,"rb+");

            Ori_data tdata;
            for(unsigned int i=0; i<(*item_temp).selected_index.size(); i++)
            {
                int id_num = (*item_temp).selected_index[i];
                fseek(binary_file,id_num*sizeof(Ori_data),SEEK_SET);  //待验证
                int num_to_read = fread(&tdata,sizeof(Ori_data),1,binary_file);

                if(num_to_read != 1)
                {
                    QWidget *temp = new QWidget;
                    QMessageBox::information(temp,"错误","原始数据2读取失败！");
                    delete temp;
                    checkout_rpy.ori_cloud2.clear();
                    return;
                }
                checkout_rpy.ori_cloud2.push_back(tdata);
            }
            fclose(binary_file);
        }

    }
}



void MainWindow::on_registration_combox_currentIndexChanged(int index)
{
    if(index==0)
    {
        registration_para.cloud1_name = nullptr;
        registration_para.cloud2_name = nullptr;
        ui->start_registrate_btn->setEnabled(false);
        return;
    }
    switch(index)
    {
    case 1:
        registration_para.registration_index = 1;
        ui->start_registrate_btn->setEnabled(true);
        ui->continune_registrate_btn->setEnabled(true);
        break;
    case 2:
        registration_para.registration_index = 2;
        ui->start_registrate_btn->setEnabled(true);
        ui->continune_registrate_btn->setEnabled(false);
        break;
    default:
        break;
    }
}

void MainWindow::on_start_registrate_btn_clicked()
{

    if(registration_para.cloud1_name == nullptr || registration_para.cloud2_name == nullptr)
    {
        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"错误","请选择两幅完整点云!");
        delete temp;
        return;
    }


    MYCloud temp;
    temp.show_index = 0;
    temp.selected_index.clear();
    cloud_seq.push_back(temp);
    add_cloud_page();

    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    (*item_cloud).cloud.points.clear();

    emit start_registrate();


}

void MainWindow::on_continune_registrate_btn_clicked()
{
    emit continue_registrate();
}

void MainWindow::on_counterBtn_triggered()
{
    static bool entry = false;

    entry = !entry;
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    if(entry)
    {
        if((*item_cloud).cloud.points.empty())
        {
            QWidget *temp = new QWidget;
            QMessageBox::information(temp,"错误","输入点云为空!");
            delete temp;
            return;
        }
        allcontourlines.contour.clear();

        long point_num = (*item_cloud).cloud.points.size();

        PointNode *pointc=new PointNode[point_num];

        for(long i=0;i<point_num;i++)
        {
            PointStorage_create_point(pointc+i, (*item_cloud).cloud.points[i].x,
                                                (*item_cloud).cloud.points[i].y,
                                                (*item_cloud).cloud.points[i].z);
        }

        float min_z = (*item_cloud).min[2];
        float max_z = (*item_cloud).max[2];

        //Preprocess the cloud points for getting contour lines
        makeSlightChange(pointc, point_num, min_z, max_z, ContourElevationUnit);

        int count = 0;
        fprintf(stderr, "computing TIN ... \n");
        TINclean(point_num, 0);
        printf("tin, point num  =%ld!!!\n", point_num);

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

        //Contour tracking starts.
        TINtriangle *t=TINget_triangle(0);
        const long TinNum=TINget_size();
        Flag flag[TinNum];
        getAllContourLines(t, flag, allcontourlines, min_z, max_z, ContourElevationUnit, TinNum);
        (*item_cloud).show_index=5;
        delete []pointc;

        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"完成","等高线已生成!");
        delete temp;
    }
    else
    {
        (*item_cloud).show_index=0;
    }
    update_GL();
}

void MainWindow::on_show_registration_all_btn_clicked()
{
    static bool entry = false;
    entry = !entry;
    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;

    if(entry)
    {
        ui->show_registration_all_btn->setText("部分拼接效果");
        if((*item_cloud).show_index==4)
            (*item_cloud).show_index = 0;

        ui->SelectBtn->setEnabled(true);
       // cout<<"hahah:12"<<endl;

    }
    else
    {
        ui->show_registration_all_btn->setText("整体拼接效果");
        if((*item_cloud).show_index==0)
            (*item_cloud).show_index = 4;

        ui->SelectBtn->setEnabled(false);
        //ui->Measure_distanceBtn->setEnabled(true);
       // cout<<"hahah:111111"<<endl;
    }
    update_GL();
}

/*
//    char *str ="hello";
//    int n=strlen(str);
//    //glRasterPos2i(0,0);
//    glRasterPos3f(0.0f, 0.0f, 0.0f);
//    for(int i=0;i<n;i++)
//    {
//        glutBitmapCharacter(GLUT_BITMAP_8_BY_13,*(str+i));
//    }

//    float c=808.2;
//    qglColor(Qt::black);
//    renderText(0.0,0.0,0.0,QString::number(c));
//    renderText(0.0,0.0,0.0,"11111");
*/




void MainWindow::on_optimizing_start_btn_clicked()
{


    double step_length[7] = {1,1,1,1,1,1,0.01};
    double err_allow_whole[7] = {0.04,0.04,0.04,0.04,0.04,0.04,0.0004};

   // fusion_para.trans_para7 = fusion.calcu_init_para(fusion_para.trans_para7, 100); /*迭代平差求解初始参数*/

    Search_result result = fusion.Powell_Search(fusion_para.trans_para7,step_length,err_allow_whole,1);

    //fusion.Find_color(result.para);

    double optimized_distance = result.optimized_distance;
    double optimized_u = result.para.u;
    double optimized_xyz[3],optimized_rpy[3];
    optimized_xyz[0] = result.para.P0.x;
    optimized_xyz[1] = result.para.P0.y;
    optimized_xyz[2] = result.para.P0.z;
    optimized_rpy[0] = result.para.roll;
    optimized_rpy[1] = result.para.pitch;
    optimized_rpy[2] = result.para.yaw;
    cout<<"optimized_distance:"<<optimized_distance<<endl;
    cout<<"optimized_xyz:"<<optimized_xyz[0]<<","<<optimized_xyz[1]<<","<<optimized_xyz[2]<<endl;
    cout<<"optimized_rpy:"<<optimized_rpy[0]<<","<<optimized_rpy[1]<<","<<optimized_rpy[2]<<endl;
    cout<<"optimized_u:"<<optimized_u<<endl;

    fusion_para.trans_para7 = result.para;

    ui->fusion_start_btn->setEnabled(true);


}

void MainWindow::on_fusion_start_btn_clicked()
{

    MYCloud temp;
    temp.show_index = 0;
    temp.selected_index.clear();
    cloud_seq.push_back(temp);
    add_cloud_page();

    fusion.Find_color(fusion_para.trans_para7);

    update_GL();
}






void MainWindow::on_input_ori_data1_btn_clicked()
{

    QString fileName = QFileDialog::getOpenFileName(this,QString("open ori_data1"));

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);

    if(fileName == NULL)
    {
        logging_model_msg << "初始文件1为空!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

        return;
    }
    else
    {
        this->checkout_rpy.binary_file_cloud1 = fileName;
        ui->check_cloud1_combox->setEnabled(true);

        logging_model_msg << "初始文件1载入成功!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

        return;
    }
}

void MainWindow::on_input_ori_data2_btn_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,QString("open ori_data2"));

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);

    if(fileName == NULL)
    {
        logging_model_msg << "初始文件2为空!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

        return;
    }
    else
    {
        this->checkout_rpy.binary_file_cloud2 = fileName;
        ui->check_cloud2_combox->setEnabled(true);

        logging_model_msg << "初始文件2载入成功!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

        return;
    }
}


void MainWindow::on_begin_checkout_btn_clicked()
{
    double step_length[3] = {1,1,1};
    double err_allow_whole[3] = {0.03,0.03,0.03};

    checkout_para.trans_para3 = this->checkout_rpy.Powell_Search(checkout_para.trans_para3,step_length,err_allow_whole,2);

    cout<<"optimized_drpy:"<<checkout_para.trans_para3.dr<<","<<checkout_para.trans_para3.dp<<","<<checkout_para.trans_para3.dy<<endl;
}

void MainWindow::on_cut_ori_data1_btn_clicked()
{
    if(!checkout_rpy.ori_cloud1.size())
    {
        QWidget *temp = new QWidget;
        QMessageBox::information(temp,"错误","数据为空!");
        delete temp;
        return;
    }

    QString filename = QFileDialog::getSaveFileName(this, QString("save ori data"), "");
    if(filename==NULL)
        return;

    QByteArray fn_txt = filename.toLocal8Bit();
    const char* ori_file_name = fn_txt.data();

    FILE* p1=NULL;

    if((p1 = fopen(ori_file_name,"wb+")) == NULL )
        printf("ERROR : ori_file cann't open .\n");

    Ori_data tdata;
    for(unsigned int i=0; i<checkout_rpy.ori_cloud1.size(); i++)
    {
        tdata = checkout_rpy.ori_cloud1[i];
        tdata.id_num = i+1;
        fwrite(&tdata,sizeof(Ori_data),1,p1);
    }
    fclose(p1);
}

void MainWindow::on_output_ori_data1_btn_clicked()
{
    if(checkout_rpy.ori_cloud1.size())
    {
        for(unsigned int i=0; i<checkout_rpy.ori_cloud1.size(); i++)
        {
            printf("xg yg zg: %lf %lf %lf, xl yl zl: %lf %lf %lf, rpy: %f %f %f \n",checkout_rpy.ori_cloud1[i].posNED[0],checkout_rpy.ori_cloud1[i].posNED[1], \
                    checkout_rpy.ori_cloud1[i].posNED[2],checkout_rpy.ori_cloud1[i].p_laser.x,checkout_rpy.ori_cloud1[i].p_laser.y,\
                    checkout_rpy.ori_cloud1[i].p_laser.z,checkout_rpy.ori_cloud1[i].euler[0],checkout_rpy.ori_cloud1[i].euler[1],\
                    checkout_rpy.ori_cloud1[i].euler[2]);
            cout<<endl;
        }
    }
}




void MainWindow::on_add_matched_point_btn_clicked()
{
//    for(vector<MYCloud>::iterator item_temp = cloud_seq.begin(); item_temp!=cloud_seq.end(); item_temp++)
//    {
//        if(fusion_para.world_cloud_name == (*item_temp).mycloud_name)
//        {
//            if((*item_temp).cloud.points.empty())
//            {
//                QWidget *temp = new QWidget;
//                QMessageBox::information(temp,"错误","世界点云为空!");
//                delete temp;
//                return;
//            }
//            if((*item_temp).selected_index.empty())
//            {
//                QWidget *temp = new QWidget;
//                QMessageBox::information(temp,"错误","世界点云还未选择匹配点!");
//                delete temp;
//                return;
//            }

//            Point p;
//            p.x = (*item_temp).cloud.points[(*item_temp).selected_index[0]].x;
//            p.y = (*item_temp).cloud.points[(*item_temp).selected_index[0]].y;
//            p.z = (*item_temp).cloud.points[(*item_temp).selected_index[0]].z;

//            fusion_para.world_point.push_back(p);
//            break;
//        }

//    }

//    for(vector<MYCloud>::iterator item_temp = cloud_seq.begin(); item_temp!=cloud_seq.end(); item_temp++)
//    {
//        if(fusion_para.pic_cloud_name == (*item_temp).mycloud_name)
//        {
//            if((*item_temp).cloud.points.empty())
//            {
//                QWidget *temp = new QWidget;
//                QMessageBox::information(temp,"错误","图像点云为空!");
//                delete temp;
//                fusion_para.world_point.pop_back();
//                return;
//            }
//            if((*item_temp).selected_index.empty())
//            {
//                QWidget *temp = new QWidget;
//                QMessageBox::information(temp,"错误","图像点云还未选择匹配点!");
//                delete temp;
//                fusion_para.world_point.pop_back();
//                return;
//            }

//            Point p;
//            p.x = (*item_temp).cloud.points[(*item_temp).selected_index[0]].x;
//            p.y = (*item_temp).cloud.points[(*item_temp).selected_index[0]].y;
//            p.z = (*item_temp).cloud.points[(*item_temp).selected_index[0]].z;

//            fusion_para.pic_point.push_back(p);
//            break;
//        }

//    }

//    int world_matched_num = fusion_para.world_point.size();
//    int pic_matched_num = fusion_para.pic_point.size();

//    if(world_matched_num == pic_matched_num)
//    {
//        QWidget *temp = new QWidget;
//        switch(world_matched_num)
//        {
//        case 1:
//            QMessageBox::information(temp,"提示","已添加一对匹配点!");
//            delete temp;
//            break;
//        case 2:
//            QMessageBox::information(temp,"提示","已添加两对匹配点!");
//            delete temp;
//            break;
//        case 3:
//            QMessageBox::information(temp,"提示","已添加三对匹配点,可开始配准!");
//            delete temp;
//            ui->add_matched_point_btn->setEnabled(false);
//            ui->optimizing_start_btn->setEnabled(true);
//            break;
//         default:
//            delete temp;
//            break;
//        }

//    }
//    else
//    {
//        QWidget *temp = new QWidget;
//        QMessageBox::information(temp,"错误","匹配点对选择错误，请重新选择!");
//        delete temp;
//        fusion_para.world_point.clear();
//        fusion_para.pic_point.clear();
//    }
}



void MainWindow::on_pushButton_clicked()
{
    Trans_para7 para;
    para = fusion.calcu_init_para(fusion_para.trans_para7, 100);

    cout<<para.P0.x<<" "<<para.P0.y<<" "<<para.P0.z<<endl;
    cout<<para.roll<<" "<<para.pitch<<" "<<para.yaw<<endl;
    cout<<para.u<<endl;
}

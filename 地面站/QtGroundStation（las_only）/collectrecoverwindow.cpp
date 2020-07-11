#include "collectrecoverwindow.h"
#include "mainwindow.h"

CollectRecoverWindow::CollectRecoverWindow(QWidget *parent) : QMainWindow(parent),ui(new Ui::CollectRecoverWindow)
{
    ui->setupUi(this);
    // ui->lineEdit->setText("");
    ui->log_view->setModel(&logging_model);
    QObject::connect(this, SIGNAL(log_Updated()), this, SLOT(updateLoggingView()));
    QObject::connect(&node,SIGNAL(update_Point()),this,SLOT(update_GL()));
}

CollectRecoverWindow::~CollectRecoverWindow()
{
    delete ui;
}

void CollectRecoverWindow::update_GL()
{

    ui->widget->updateGL();

}

void CollectRecoverWindow::updateLoggingView()
{
    ui->log_view->scrollToBottom();
}

void CollectRecoverWindow::DrawPoints()
{
    vorxba = (max_x+min_x)/2;
    voryba = (max_y+min_y)/2;
    vorzba = (max_z+min_z)/2;

    glPointSize(2.0);
    glBegin( GL_POINTS);

    for(unsigned int i=0;i<points_recv_temp.points.size();i++)
    {

        if(ColorMode)
        {
            float highZ = points_recv_temp.points[i].z-min_z;
            float boundingBoxZ = max_z-min_z;

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
            glVertex3f(points_recv_temp.points[i].x-vorxba,points_recv_temp.points[i].y-voryba,points_recv_temp.points[i].z-vorzba);
        }
        else
        {
            glColor3f(0.0f,0.0f,0.0f);
            glVertex3f(points_recv_temp.points[i].x-vorxba,points_recv_temp.points[i].y-voryba,points_recv_temp.points[i].z-vorzba);
        }

    }

    glEnd();


}

void CollectRecoverWindow::on_ReturnBtn_clicked()
{

    ColorMode = 0;
    selectMode = 0;
    vorxba = 0;voryba = 0;vorzba = 0;
    max_x = -10000;min_x = 10000;max_y = -10000;min_y = 10000;max_z = -10000;min_z = 10000;
    window_selected = 0;

    points_recv_temp.points.clear();
    points_recv.points.clear();

    exec_flag = 0;

    num = 0;
    parentWidget()->show();
    this->hide();
}

void CollectRecoverWindow::on_openfileBtn_clicked()
{
    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);

    fileName = QFileDialog::getOpenFileName(this,QString("open data"));
    if(fileName == NULL)
    {
        logging_model_msg << "open file failed!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

        return;
    }
    else
    {
        logging_model_msg << "open file succeed!";
        QVariant new_row(QString(logging_model_msg.str().c_str()));
        logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
        emit log_Updated(); // used to readjust the scrollbar

        return;
    }
}

void CollectRecoverWindow::on_runCore_offline_Btn_clicked()
{
    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"roscore ; exec bash\"");
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "roscore running!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}

void CollectRecoverWindow::on_runCore_online_Btn_clicked()
{
    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"roscore ; exec bash\"");
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "roscore running!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}

void CollectRecoverWindow::on_getDataBtn_clicked()
{
    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"roslaunch lidar_station lidar_online.launch ; exec bash\"");
   // system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "lidar_station_online running!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}

void CollectRecoverWindow::on_posDetectBtn_clicked()
{
    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"rqt_plot /gps_packet/ned_xyz\"");
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "lidar position detecting!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}

void CollectRecoverWindow::on_eulerDetectBtn_clicked()
{
    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"rqt_plot /imu_packet/imu_euler\"");
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "lidar pose detecting!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}

void CollectRecoverWindow::on_getPointBtn_clicked()
{
    QByteArray fn=(fileName).toLocal8Bit();
    const char* file_name=fn.data();
    cout<<file_name<<endl;


    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"roslaunch lidar_station lidar_offline.launch file-name:=%s; exec bash\"",file_name);
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);


    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "lidar_station_offline running!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}



void CollectRecoverWindow::on_point_show_Btn_clicked()
{
    exec_flag = 1;
    node.init();

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "showing points!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar
}

void CollectRecoverWindow::on_save_point_Btn_clicked()
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
    strcat(filetxtname,"_ori_point.txt");
    strcat(filelasname,"_ori_point.las");

    FILE* p1=NULL;
    p1 = fopen(filetxtname,"a+");
    for(unsigned int i=0; i<points_recv.points.size(); i++)
    {
        fprintf(p1,"%lf %lf %lf \n",points_recv.points[i].x,points_recv.points[i].y,points_recv.points[i].z);
    }
    fclose(p1);

    char str[512];
    sprintf(str,"gnome-terminal -x bash -c \"txt2las %s -o %s; exec bash\"",filetxtname,filelasname);

    QProcess *pro=new QProcess();
    pro->start(str);

    std::stringstream logging_model_msg;
    logging_model.insertRows(logging_model.rowCount(),1);
    logging_model_msg << "point cloud saving!";
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    emit log_Updated(); // used to readjust the scrollbar

}

void CollectRecoverWindow::on_Color_change_Btn_clicked()
{
    ColorMode++;
    ColorMode = ColorMode%2;
    ui->widget->updateGL();
}

void CollectRecoverWindow::on_connect_Btn_clicked()
{
    char str[180];
    sprintf(str,"gnome-terminal -x bash -c \"cd ; exec bash\"");
    //system(str);
    QProcess *pro=new QProcess();
    pro->start(str);
}

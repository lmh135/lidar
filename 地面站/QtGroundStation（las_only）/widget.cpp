#include<GL/glu.h>
#include "widget.h"
#include <QString>

#include<qevent.h>
#include<GL/glut.h>
#include<iostream>
#include<QGridLayout>
#include <QVBoxLayout>
using namespace std;

//pcl::PointXYZ nearestPoint;

int window_selected = 0;

Widget::Widget(QWidget* parent,
               const QString title
               ):QGLWidget(parent)

{
    setWindowTitle("OpenglWindow");

}

Widget::~Widget(void)
{
}

void Widget::initializeGL()
{
    glClearColor( 1.0, 1.0, 1.0, 0.0 );
    glShadeModel(GL_SMOOTH);
    glClearDepth( 1.0 );
    glEnable( GL_DEPTH_TEST );  
    glViewport(0, 0,this->width(),this->height());

    //光照
    float intensity[] = {1,1,1,1};
    float position[] = {1,1,5,0}; // directional behind the viewer
    glLightfv(GL_LIGHT0,GL_DIFFUSE,intensity);
    glLightfv(GL_LIGHT0,GL_SPECULAR,intensity);
    glLightfv(GL_LIGHT0,GL_POSITION,position);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
}

void Widget::resizeGL(int width, int height)
{
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );//zuobiao->0  
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );
}

void Widget::paintGL()
{

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    //gluPerspective( 45.0, 1280/640, 0.1, 100 );

//    glPointSize(20.0);
//    glColor3f(0.0f,0.0f,0.0f);
//    glVertex3f(3.0f,3.0f,0.0f);
//    glVertex3f(3.0f,3.0f,5.0f);
//    glVertex3f(0.0f,0.0f,0.0f);
//    if(window_selected == 0)
//    {


//    }
    if(window_selected == 1)
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity( );
        gluPerspective( 80,this->width()/this->height(), 0.0625f, 1500.0f );

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        //glTranslatef(0.0f,0.0f,-20.0f);

        glTranslatef(translateX,translateY,-50.0f+numSteps);//沿XYZ坐标轴平移
        glRotatef(180+rotationX, 1.0, 0.0, 0.0);//沿X轴旋转
        glRotatef(rotationY, 0.0, 1.0, 0.0);
        glRotatef(rotationZ, 0.0, 0.0, 1.0);

        ShowWindow showwindow;
        showwindow.DrawLasPoints();
    }
    if(window_selected == 2)
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity( );
        gluPerspective( 80,this->width()/this->height(), 0.0625f, 1500.0f );

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        //glTranslatef(0.0f,0.0f,-20.0f);

        glTranslatef(translateX,translateY,-50.0f+numSteps);//沿XYZ坐标轴平移
        glRotatef(180+rotationX, 1.0, 0.0, 0.0);//沿X轴旋转
        glRotatef(rotationY, 0.0, 1.0, 0.0);
        glRotatef(rotationZ, 0.0, 0.0, 1.0);

        CollectRecoverWindow collectrecoverwindow;
        collectrecoverwindow.DrawPoints();
    }
    if(window_selected == 3)
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity( );
        gluPerspective( 80,this->width()/this->height(), 0.0625f, 1500.0f );

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        //glTranslatef(0.0f,0.0f,-20.0f);

        glTranslatef(translateX,translateY,-50.0f+numSteps);//沿XYZ坐标轴平移
        glRotatef(180+rotationX, 1.0, 0.0, 0.0);//沿X轴旋转
        glRotatef(rotationY, 0.0, 1.0, 0.0);
        glRotatef(rotationZ, 0.0, 0.0, 1.0);

        glViewport(0,this->height()/2,this->width()/2,this->height()/2);
        FilterWindow filterwindow;
        filterwindow.Draw_unfilted_Points();

        glViewport(0,0,this->width()/2,this->height()/2);
        filterwindow.Draw_filted_Points();

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity( );
        gluPerspective( 80,this->width()/(2*this->height()), 0.0625f, 1500.0f );

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glTranslatef(translateX,translateY,-50.0f+numSteps);//沿XYZ坐标轴平移
        glRotatef(180+rotationX, 1.0, 0.0, 0.0);//沿X轴旋转
        glRotatef(rotationY, 0.0, 1.0, 0.0);
        glRotatef(rotationZ, 0.0, 0.0, 1.0);

        glViewport(this->width()/2,0,this->width()/2,this->height());
        filterwindow.Draw_triangulations();

    }



    if(window_selected == 6)
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity( );
        gluPerspective( 80,this->width()/this->height(), 0.0625f, 1500.0f );

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        //glTranslatef(0.0f,0.0f,-20.0f);

        glTranslatef(translateX,translateY,-50.0f+numSteps);//沿XYZ坐标轴平移
        glRotatef(180+rotationX, 1.0, 0.0, 0.0);//沿X轴旋转
        glRotatef(rotationY, 0.0, 1.0, 0.0);
        glRotatef(rotationZ, 0.0, 0.0, 1.0);

        registration reg_window;
        reg_window.Drawcloud1();
    }

}

void Widget::wheelEvent(QWheelEvent *event)//滑轮缩放
{
    int numDegrees=event->delta()/8;
    numSteps+=numDegrees/15;
    updateGL();
}

void Widget::mousePressEvent(QMouseEvent *event)//鼠标按住
{
    lastPos = event->pos();
    if(Qt::LeftButton == event->button())
    {
        pressPos = event->pos();
     //   cout<<"widget press: "<<pressPos.x()<<" "<<pressPos.y()<<endl;
        //screen2GLPoint();
        updateGL();
    }

}

void Widget::mouseReleaseEvent(QMouseEvent *event)
{
    if(Qt::LeftButton == event->button())
    {
        releasePos = event->pos();
       // cout<<"widget release: "<<releasePos.x()<<" "<<releasePos.y()<<endl;
        if(selectMode)
            emit release_left_mouse();
    }
}

void Widget::mouseMoveEvent(QMouseEvent *event)//鼠标移动
{
    if(selectMode == 1)
        return;

    GLfloat dx=GLfloat(lastPos.x()-event->x())/1280;
    GLfloat dy=GLfloat(event->y()-lastPos.y())/640;
    if (event->buttons() & Qt::LeftButton)
    {
        rotationX += 180 * dy;
        rotationY += 180 * dx;
    }

    else if (event->buttons() & Qt::RightButton)
    {
        translateX -= 100*dx;
        translateY -= 100*dy;
    }
    // cout<<" rotx:"<<rotationX<<" roty:"<<rotationY<<" dx:"<<dx<<" dy:"<<dy<<endl;
    lastPos = event->pos();
    updateGL();
}


/*
void Widget::screen2GLPoint()
{
    int x = lastPos.x();    // 屏幕坐标
    int y = lastPos.y();
    GLint viewport[4];
    GLdouble mvmatrix[16], projmatrix[16];
    GLfloat winx, winy, winz;
    GLdouble posx, posy, posz;
    glPushMatrix();
    //glScalef(0.1, 0.1, 0.1);
    glGetIntegerv(GL_VIEWPORT, viewport);   // 获取三个矩阵
    glGetDoublev(GL_MODELVIEW_MATRIX, mvmatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, projmatrix);
    glPopMatrix();
    winx = x;
    winy = this->height() - y;
    glReadPixels((int)winx, (int)winy, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winz);   // 获取深度
    gluUnProject(winx, winy, winz, mvmatrix, projmatrix, viewport, &posx, &posy, &posz); // 获取三维坐标

    posx = posx+vorxba;
    posy = posy+voryba;
    posz = posz+vorzba;

    if(selectMode==1)
    {
        if(((posx)>=min_x)&&((posx)<=max_x)
            &&((posy)>=min_y)&&((posy)<=max_y)
            &&((posz)>=min_z)&&((posz)<=max_z))
        {
            pcl::PointXYZ searchPoint;

            double min_distance = 1;
            std::vector<int> pointIdxVec;
            searchPoint.x = posx;
            searchPoint.y = posy;
            searchPoint.z = posz;

            if (octree.voxelSearch (searchPoint, pointIdxVec))
            {
                for (unsigned int i = 0; i < pointIdxVec.size (); i++)
                {
                    double temp_distance;
                    pcl::PointXYZ p = cloud_in.points[pointIdxVec[i]];
                    temp_distance = (posx-p.x)*(posx-p.x)+(posy-p.y)*(posy-p.y)+(posz-p.z)*(posz-p.z);
                    if(temp_distance<min_distance)
                    {
                        min_distance = temp_distance;
                        nearestPoint = p;
                    }
                }
                emit selected_ok();
                return;
            }

        }

    }

    return;
}

*/

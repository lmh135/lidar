#include<GL/glu.h>
#include "widget3.h"
#include <QString>

#include<qevent.h>
#include<GL/glut.h>
#include<iostream>
#include<QGridLayout>
#include <QVBoxLayout>
using namespace std;

//pcl::PointXYZ nearestPoint;



Widget3::Widget3(QWidget* parent,
               const QString title
               ):QGLWidget(parent)

{
    setWindowTitle("OpenglWindow3");

}

Widget3::~Widget3(void)
{
}

void Widget3::initializeGL()
{
    glClearColor( 1.0, 1.0, 1.0, 0.0 );
    glShadeModel(GL_SMOOTH);
    glClearDepth( 1.0 );
    glEnable( GL_DEPTH_TEST );  
    glViewport(0, 0,this->width(),this->height());
}

void Widget3::resizeGL(int width, int height)
{
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity( );//zuobiao->0  
    glMatrixMode( GL_MODELVIEW );
    glLoadIdentity( );
}

void Widget3::paintGL()
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
        reg_window.Drawcloud3();
    }

}

void Widget3::wheelEvent(QWheelEvent *event)//滑轮缩放
{
    int numDegrees=event->delta()/8;
    numSteps+=numDegrees/15;
    updateGL();
}

void Widget3::mousePressEvent(QMouseEvent *event)//鼠标按住
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

void Widget3::mouseReleaseEvent(QMouseEvent *event)
{
    if(Qt::LeftButton == event->button())
    {
        releasePos = event->pos();
       // cout<<"widget release: "<<releasePos.x()<<" "<<releasePos.y()<<endl;
        if(selectMode)
            emit release_left_mouse();
    }
}

void Widget3::mouseMoveEvent(QMouseEvent *event)//鼠标移动
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






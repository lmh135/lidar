#include "widget.h"

Widget::Widget(QWidget* parent,
               const QString title
               ):QGLWidget(parent)

{
    setWindowTitle("OpenglWindow");
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
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
    float position[] = {1,1,5,0};
    glLightfv(GL_LIGHT0,GL_DIFFUSE,intensity);
    glLightfv(GL_LIGHT0,GL_SPECULAR,intensity);
    glLightfv(GL_LIGHT0,GL_POSITION,position);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
}

void Widget::resizeGL(int width, int height)
{
    glViewport(0, 0,this->width(),this->height());
    glMatrixMode(GL_PROJECTION);//修改投影矩阵
    glLoadIdentity();//导入单位阵
    gluPerspective( 100,2, 0.0625f, 1500.0f );
    glMatrixMode(GL_MODELVIEW);//修改模型视图
    glLoadIdentity();
}

void Widget::paintGL()
{

    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

    glViewport(0, 0,this->width(),this->height());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity( );
    gluPerspective( 100,2, 0.0625f, 1500.0f );
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glTranslatef(translateX,translateY,-50.0f+numSteps);//沿XYZ坐标轴平移
    glRotatef(180+rotationX, 1.0, 0.0, 0.0);//沿X轴旋转
    glRotatef(rotationY, 0.0, 1.0, 0.0);
    glRotatef(rotationZ, 0.0, 0.0, 1.0);

    vector<MYCloud>::iterator item_cloud = cloud_seq.begin();
    item_cloud += tab_current_index;
    if((*item_cloud).show_index==5)
    {
        if((*item_cloud).cloud.points.empty())
            return;

        float min_x = (*item_cloud).min[0];
        float max_x = (*item_cloud).max[0];

        float min_y = (*item_cloud).min[1];
        float max_y = (*item_cloud).max[1];

        float min_z = (*item_cloud).min[2];
        float max_z = (*item_cloud).max[2];

        float ave_x = (max_x+min_x)/2;
        float ave_y = (max_y+min_y)/2;
        float ave_z = (max_z+min_z)/2;

        glEnable(GL_LINE_SMOOTH);
        glColor3f(0.0f,0.0f,0.0f);

        std::multimap<Properties, ContourLine>::iterator it;
        std::vector<std::vector<ContourPoint>>::iterator it1;
        std::vector<ContourPoint>::iterator it2;

        for (it=allcontourlines.contour.begin(); it!=allcontourlines.contour.end(); ++it)
        {
               double elevation= (*it).first.elevation;
               for(it1=(*it).second.cl.begin(); it1!=(*it).second.cl.end(); ++it1)
               {
                   glBegin(GL_LINE_STRIP);
                   glLineWidth(3.0);
                   glColor3f(0.0f,0.0f,0.0f);

                   for(it2=(*it1).begin();it2!=(*it1).end();++it2)
                   {
                       //glVertex3f( (*it2).x-ave_x,(*it2).y-ave_y, elevation-ave_z);
                       //glVertex2f( (*it2).x-ave_x,(*it2).y-ave_y);
                       glVertex3f( (*it2).x-ave_x,(*it2).y-ave_y, 0.0f);
                   }
                   glEnd();
                   it2=(*it1).begin();
                   qglColor(Qt::blue);
                   renderText((*it2).x-ave_x-1.0,(*it2).y-ave_y-0.5,0.0,QString::number(elevation)+"m");
               }
        }

        glBegin(GL_LINE_STRIP);
        glLineWidth(1.0);
        glColor3f(0.0f,0.0f,0.0f);
        glVertex3f( min_x-ave_x-5,max_y-ave_y+5, 0.0f);
        glVertex3f( min_x-ave_x-10,max_y-ave_y+5, 0.0f);
        glEnd();
        renderText( min_x-ave_x-11,max_y-ave_y+3, 0.0f,"5m");

        glBegin(GL_LINE_STRIP);
        glLineWidth(1.0);
        glColor3f(0.0f,0.0f,0.0f);
        glVertex3f( min_x-ave_x-5,max_y-ave_y+4.8, 0.0f);
        glVertex3f( min_x-ave_x-5,max_y-ave_y+5.2, 0.0f);
        glEnd();

        glBegin(GL_LINE_STRIP);
        glLineWidth(1.0);
        glColor3f(0.0f,0.0f,0.0f);
        glVertex3f( min_x-ave_x-10,max_y-ave_y+4.8, 0.0f);
        glVertex3f( min_x-ave_x-10,max_y-ave_y+5.2, 0.0f);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor3f(0.0f,0.0f,0.0f);
        glVertex3f(min_x-ave_x-11,max_y-ave_y+4, 0.0f);
        glVertex3f(min_x-ave_x-11,max_y-ave_y+6, 0.0f);
        glVertex3f(min_x-ave_x-4,max_y-ave_y+6, 0.0f);
        glVertex3f(min_x-ave_x-4,max_y-ave_y+4, 0.0f);
        glEnd();

    }
    draw_fuc();

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
        updateGL();
    }

}

void Widget::mouseReleaseEvent(QMouseEvent *event)
{
    if(Qt::LeftButton == event->button())
    {
        releasePos = event->pos();
        if(select_mode)
            emit release_left_mouse();
    }
}

void Widget::mouseMoveEvent(QMouseEvent *event)//鼠标移动
{
    if(select_mode)
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
        rotationZ += 180 * dx;
       // translateX -= 100*dx;
       // translateY -= 100*dy;
    }
    lastPos = event->pos();
    updateGL();
}

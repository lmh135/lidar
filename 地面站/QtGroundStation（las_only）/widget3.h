#ifndef WIDGET3_H
#define WIDGET3_H

#include <QWidget>
#include <QGLWidget>
#include <qgl.h>
#include <QLineEdit>
#include <QPushButton>
#include"showwindow.h"
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_search.h>
#include "collectrecoverwindow.h"
#include "filterwindow.h"
#include "test.h"
#include "registration.h"

class Widget3 : public QGLWidget
{
    Q_OBJECT
public:
    Widget3(QWidget* parent=0,
           const QString title =0
           );
    ~Widget3(void);
    QPoint lastPos;
    QPoint pressPos;
    QPoint releasePos;


public slots:
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
   // void screen2GLPoint();

signals:

    void release_left_mouse();

protected:
    void initializeGL(void);
    void resizeGL(int width, int height);
    void paintGL(void);

private:
    GLint numSteps=0;

    GLfloat rotationX=0.0;
    GLfloat rotationY=0.0;
    GLfloat rotationZ=0.0;
    GLfloat translateX=0.0;
    GLfloat translateY=0.0;

};


extern int window_selected;
extern int selectMode;


#endif // WIDGET3_H

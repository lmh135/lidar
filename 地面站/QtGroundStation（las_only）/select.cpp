#include "select.h"
#include<GL/glu.h>
#include<GL/glut.h>
#include <stdio.h>

//      ----    Buffers For Selection   ----        //
static GLuint *select_buffer = NULL;               //OpenGL select mode buffer
static unsigned int select_buffer_size = 0;




void Selection_init(pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud)
{
    int point_num = p_cloud->points.size();
    select_buffer_size = point_num * 5;
    select_buffer = (GLuint *)malloc(sizeof(GLuint)*select_buffer_size);

}
void Selection_destroy(void)
{
    free(select_buffer);
    select_buffer_size = 0;
}

vector<int> GetSelectedIndex(unsigned int hits)
{
    vector<int> index;
    unsigned int i;
    for(i = 0; i < hits; i++)
    {
        int hit_id = *(select_buffer+i*4+3);
        index.push_back(hit_id);
    }
    return index;
}

int DoOpenGLSelection(float start_x, float start_y, float current_x, float current_y, pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud,double ave_x,double ave_y,double ave_z)
{
    float center_x;
    float center_y;
    float select_width;
    float select_height;



    GLint viewport[4];
    glGetIntegerv(GL_VIEWPORT,viewport);

    //if select rectangulate is just a point
    if (start_x == current_x && start_y == current_y)
    {
        center_x = start_x;
        center_y = viewport[3] - start_y;
        select_width = 4;
        select_height = 4;
    }
    else
    {
        center_x = ( start_x + current_x )/2;
        center_y = viewport[3] - ( start_y + current_y )/2;
        if(current_x - start_x > 0)
            select_width = (current_x - start_x);
        else
            select_width = (start_x - current_x);

        if(current_y - start_y > 0)
            select_height = (current_y - start_y);
        else
            select_height = (start_y - current_y);
    }
    //need??TODO    make sure?
    if(select_width == 0)
        select_width = 4;
    if(select_height == 0)
        select_height = 4;

    //printf("center_x = %f\ncenter_y = %f\n select_width = %f\n select_height = %f\n" , center_x, center_y, select_width, select_height);
    //printf("start_x = %f\nstart_y = %f\n current_x = %f\n current_y = %f\n" , start_x, start_y, current_x, current_y);

    //do select
    //printf("the size of select buffer is %d" , s_gl_select_buffer_size);
    glSelectBuffer(select_buffer_size, select_buffer);
    (void)glRenderMode(GL_SELECT);			//why void???
    glInitNames();
    glPushName(-1);					//why -1?

    double mp[16];

    glMatrixMode(GL_PROJECTION);
    glGetDoublev(GL_PROJECTION_MATRIX ,mp);
    glPushMatrix();
    glLoadIdentity();
    gluPickMatrix(center_x, center_y, select_width, select_height, viewport);
    glMultMatrixd(mp);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();


    if(p_cloud->points.empty())					//better to make the "edit" button insensitive TODO
    {
        //warning there are no points
        return 1;
    }

    pcl::PointXYZ point;
    int point_num = p_cloud->points.size();
//            printf("@@@@@@@@@@@@@@ point_num is %d\n",point_num);

    int i;
    for(i = 0;i < point_num; i++)
    {
        point = p_cloud->points[i];
        //if (point->delete_sn == 0)
        {
            glLoadName(i);
            glBegin(GL_POINTS);
            glVertex3f(point.x-ave_x,point.y-ave_y,point.z-ave_z);
            glEnd();
        }
    }

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    unsigned int hits;
    hits = glRenderMode(GL_RENDER);


    return hits;
}

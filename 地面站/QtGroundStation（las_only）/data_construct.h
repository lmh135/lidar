#ifndef DATA_CONSTRUCT_H
#define DATA_CONSTRUCT_H

#define share_num 50


typedef struct _Point
{
    double x;
    double y;
    double z;
}Point;


typedef struct _Points
{
    Point p[share_num][192];
    int id_num;
    int flag;
}Points;










#endif // DATA_CONSTRUCT_H

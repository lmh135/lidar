#ifndef CONTOUR_H
#define CONTOUR_H
#include"triangulate.h"
#include"points_array.h"
#include<vector>
#include<map>
#include<algorithm>
#include<iostream>
#include<cmath>
#include<decimal/decimal>
struct Flag 
{
   int TriangleFlag;
};

class Properties
{
public:
    float elevation;
    Properties()
    {
        elevation=0;
    }
    Properties(float f)
    {
        elevation=f;
    }
    bool operator < (const Properties& p)const
    {
        if(elevation < p.elevation)
        {
            return true;
        }
        else
            return false;
    }
};
class ContourPoint
{
public:
    float x;
    float y;
    ContourPoint(){x=0;y=0;}
    ContourPoint(float x, float y){this->x=x; this->y=y;}
    ContourPoint(const ContourPoint &p){x=p.x;y=p.y;}
    ~ContourPoint(){;}
    ContourPoint& operator = (const ContourPoint &p)
    {
        x=p.x;
        y=p.y;
        return *this;
    }
    bool operator == (const ContourPoint &p) const
    {
        float AcceptedError=0.0001;
        std::decimal::decimal32 ae=std::decimal::decimal32(AcceptedError);
        if(std::decimal::decimal32(fabs(x-p.x))<=ae&&
               std::decimal::decimal32(fabs(y-p.y))<=ae)
            return true;
        else 
            return false;
    }    
    bool operator != (const ContourPoint &p) const
    {
        float AcceptedError=0.0001;
        std::decimal::decimal32 ae=std::decimal::decimal32(AcceptedError);
        if(std::decimal::decimal32(fabs(x-p.x))>ae||
           std::decimal::decimal32(fabs(x-p.x))>ae)
            return true;
        else 
            return false;
    }
    friend std::ostream & operator << (std::ostream &os, const ContourPoint &obj )
    {
        os <<"("<< obj.x <<", "<<obj.y<<")"<<std::endl;
        return os;
    }
};

class ContourLine
{
public:
    std::vector<std::vector<ContourPoint>> cl;
    int getContourLinefromBoundary(TINtriangle *t, Flag *m, float z);
    int getContourLinefromInside(TINtriangle *t, Flag *m, float z);
    void filterNoise(int ThresholdPointNum);
};

class Contour
{
public:
    std::multimap<Properties, ContourLine> contour;


};

void findBoundary(TINtriangle *t, Flag *m);
void setTriangleFlag(TINtriangle *t1, Flag *m1, float z, long TinNum);
float getContourPoint(const float& x0, const float& x1, const float& z0, const float& z1, const float& z);
int NotIn(int a, int b, int c);
float getZmax(PointNode* p, long PointNum);
float getZmin(PointNode* p, long PointNum);
void makeSlightChange(PointNode* p, long PointNum, float Zmin, float Zmax, const float unit);
void getAllContourLines(TINtriangle* t, Flag* f, Contour& c, float Zmin, float Zmax, const float unit, long TinNum);
#endif

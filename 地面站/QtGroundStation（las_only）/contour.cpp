#include"contour.h"
static int ContourNum=0;
int ContourLine::getContourLinefromBoundary(TINtriangle *t, Flag *m, float z)
{
    ContourPoint* pP1=new ContourPoint;
    float* px0=new float;
    float* px1=new float;
    float* py0=new float;
    float* py1=new float;
    float* pz0=new float;
    float* pz1=new float;
    long i=0;
    int BoundaryContourNum=0;
    for(; i<TINget_size(); i++)
    {
        if((m+i)->TriangleFlag==100)
        {
            /*Initialization*/
            int PointNum=0;
            long j=i;
            //It can decide the contour line comes from which neighbor triangle.
            int InAngle=0;
            //It can decide the contour line goes to which neighbor triangle.
            int OutAngle=0;
            //It is the index number of the next triangle.
            long NextTriangle=0;
            //1-2-boundary
            *px0=*((t+j)->V[1]+0);
            *px1=*((t+j)->V[2]+0);
            *py0=*((t+j)->V[1]+1);
            *py1=*((t+j)->V[2]+1);
            *pz0=*((t+j)->V[1]+2);
            *pz1=*((t+j)->V[2]+2);
            *pP1=ContourPoint
               (getContourPoint
                (*px0, *px1, *pz0, *pz1, z),
                getContourPoint
                (*py0, *py1, *pz0, *pz1, z));
            (m+j)->TriangleFlag=0;
            InAngle=(t+j)->N[0]%3;
            OutAngle=0;
            NextTriangle=(t+j)->N[OutAngle]/3;

            /*Track the contour line among the triangles*/
            std::vector<ContourPoint> ClTemp;
            bool Boundary=false;
            while(!Boundary)
            {
                ClTemp.push_back((*pP1));
                PointNum++;
                j=NextTriangle;
                switch ((m+j)->TriangleFlag)
                {
                    //Reach the boundary
                    case 100: Boundary=true;
                              (m+j)->TriangleFlag=0;
                              break;
                    //0-1, 0-2
                    case 3:
                            if(InAngle==1)
                            {
                                *px0=*((t+j)->V[0]+0);
                                *px1=*((t+j)->V[1]+0);
                                *py0=*((t+j)->V[0]+1);
                                *py1=*((t+j)->V[1]+1);
                                *pz0=*((t+j)->V[0]+2);
                                *pz1=*((t+j)->V[1]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(1, 2, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else if(InAngle==2)
                            {
                                *px0=*((t+j)->V[0]+0);
                                *px1=*((t+j)->V[2]+0);
                                *py0=*((t+j)->V[0]+1);
                                *py1=*((t+j)->V[2]+1);
                                *pz0=*((t+j)->V[0]+2);
                                *pz1=*((t+j)->V[2]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(1, 2, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else
                                std::cout<< "Boundary Error!"<< std::endl;
                            break;
                    //0-1, 1-2
                    case 5: if(InAngle==0)
                            {
                                *px0=*((t+j)->V[0]+0);
                                *px1=*((t+j)->V[1]+0);
                                *py0=*((t+j)->V[0]+1);
                                *py1=*((t+j)->V[1]+1);
                                *pz0=*((t+j)->V[0]+2);
                                *pz1=*((t+j)->V[1]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(0, 2, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                             }
                            else if(InAngle==2)
                            {
                                *px0=*((t+j)->V[1]+0);
                                *px1=*((t+j)->V[2]+0);
                                *py0=*((t+j)->V[1]+1);
                                *py1=*((t+j)->V[2]+1);
                                *pz0=*((t+j)->V[1]+2);
                                *pz1=*((t+j)->V[2]+2);
                                *pP1=ContourPoint
                                    (getContourPoint
                                     (*px0, *px1, *pz0, *pz1, z),
                                     getContourPoint
                                     (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(0, 2, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else
                                std::cout<< "Boundary Error!"<< std::endl;
                            break;
                    //0-2, 1-2
                    case 6: if(InAngle==0)
                            {
                                *px0=*((t+j)->V[0]+0);
                                *px1=*((t+j)->V[2]+0);
                                *py0=*((t+j)->V[0]+1);
                                *py1=*((t+j)->V[2]+1);
                                *pz0=*((t+j)->V[0]+2);
                                *pz1=*((t+j)->V[2]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(0, 1, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else if(InAngle==1)
                            {
                                *px0=*((t+j)->V[1]+0);
                                *px1=*((t+j)->V[2]+0);
                                *py0=*((t+j)->V[1]+1);
                                *py1=*((t+j)->V[2]+1);
                                *pz0=*((t+j)->V[1]+2);
                                *pz1=*((t+j)->V[2]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(0, 1, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else
                                std::cout<< "Boundary Error!"<< std::endl;
                            break;
                     case 0: Boundary=true;
                             std::cout<<"Error occurs in the Boundary Function"<<std::endl;
                             break;
                     default:std::cout<<"Error occurs in the Boundary Function"<<std::endl;
                             break;
                }
            }
            std::cout<< "The " << BoundaryContourNum
                     << " Boundary Contour Line's Point Number Is : "
                     << PointNum << std::endl;
            BoundaryContourNum++;
            ContourNum++;//move to the next contour line
            cl.push_back(ClTemp);
        }
    }
    std::cout<<"Boundary Contours' Number Is: "
             <<BoundaryContourNum<<std::endl;
    delete pP1;
    delete px0;
    delete px1;
    delete py0;
    delete py1;
    delete pz0;
    delete pz1;
    return BoundaryContourNum;
}

int ContourLine::getContourLinefromInside(TINtriangle *t, Flag *m, float z)
{
    ContourPoint* pP1=new ContourPoint;
    float* px0=new float;
    float* px1=new float;
    float* py0=new float;
    float* py1=new float;
    float* pz0=new float;
    float* pz1=new float;
    long i=0;
    int InsideContourNum=0;
    for(; i<TINget_size(); i++)
    {
        if((m+i)->TriangleFlag<100 && (m+i)->TriangleFlag>0)
        { 
            /*Initialization*/
            int PointNum=0;
            long j=i;
            //It can decide the contour line comes from which neighbor triangle.
            int InAngle=0;
            //It can decide the contour line goes to which neighbor triangle.
            int OutAngle=0;
            //It is the index number of the next triangle.
            long NextTriangle=0;
            std::vector<ContourPoint> ClTemp;
            switch ((m+j)->TriangleFlag)
            {
                //0-1, 0-2
                case 3: *px0=*((t+j)->V[0]+0);
                        *px1=*((t+j)->V[1]+0);
                        *py0=*((t+j)->V[0]+1);
                        *py1=*((t+j)->V[1]+1);
                        *pz0=*((t+j)->V[0]+2);
                        *pz1=*((t+j)->V[1]+2);
                        *pP1=ContourPoint
                             (getContourPoint
                              (*px0, *px1, *pz0, *pz1, z),
                              getContourPoint
                              (*py0, *py1, *pz0, *pz1, z));
                        ClTemp.push_back(*pP1);
                        PointNum++;
                        *px0=*((t+j)->V[0]+0);
                        *px1=*((t+j)->V[2]+0);
                        *py0=*((t+j)->V[0]+1);
                        *py1=*((t+j)->V[2]+1);
                        *pz0=*((t+j)->V[0]+2);
                        *pz1=*((t+j)->V[2]+2);
                        *pP1=ContourPoint
                             (getContourPoint
                              (*px0, *px1, *pz0, *pz1, z),
                              getContourPoint
                              (*py0, *py1, *pz0, *pz1, z));
                       // ClTemp.push_back(*pP1);
                        (m+j)->TriangleFlag=0;
                        OutAngle=1;
                        NextTriangle=(t+j)->N[OutAngle]/3;
                        InAngle=(t+j)->N[OutAngle]%3;
                        break;

                //0-1, 1-2
                case 5: *px0=*((t+j)->V[0]+0);
                        *px1=*((t+j)->V[1]+0);
                        *py0=*((t+j)->V[0]+1);
                        *py1=*((t+j)->V[1]+1);
                        *pz0=*((t+j)->V[0]+2);
                        *pz1=*((t+j)->V[1]+2);
                        *pP1=ContourPoint
                             (getContourPoint
                              (*px0, *px1, *pz0, *pz1, z),
                              getContourPoint
                              (*py0, *py1, *pz0, *pz1, z));
                        ClTemp.push_back(*pP1);
                        PointNum++;
                        *px0=*((t+j)->V[1]+0);
                        *px1=*((t+j)->V[2]+0);
                        *py0=*((t+j)->V[1]+1);
                        *py1=*((t+j)->V[2]+1);
                        *pz0=*((t+j)->V[1]+2);
                        *pz1=*((t+j)->V[2]+2);
                        *pP1=ContourPoint
                             (getContourPoint
                              (*px0, *px1, *pz0, *pz1, z),
                              getContourPoint
                              (*py0, *py1, *pz0, *pz1, z));
                        //ClTemp.push_back(*pP1);
                        (m+j)->TriangleFlag=0;
                        OutAngle=0;
                        NextTriangle=(t+j)->N[OutAngle]/3;
                        InAngle=(t+j)->N[OutAngle]%3;
                        break;
                //0-2, 1-2
                case 6: *px0=*((t+j)->V[1]+0);
                        *px1=*((t+j)->V[2]+0);
                        *py0=*((t+j)->V[1]+1);
                        *py1=*((t+j)->V[2]+1);
                        *pz0=*((t+j)->V[1]+2);
                        *pz1=*((t+j)->V[2]+2);
                        *pP1=ContourPoint
                             (getContourPoint
                              (*px0, *px1, *pz0, *pz1, z),
                              getContourPoint
                              (*py0, *py1, *pz0, *pz1, z));
                        ClTemp.push_back(*pP1);
                        PointNum++;
                        *px0=*((t+j)->V[0]+0);
                        *px1=*((t+j)->V[2]+0);
                        *py0=*((t+j)->V[0]+1);
                        *py1=*((t+j)->V[2]+1);
                        *pz0=*((t+j)->V[0]+2);
                        *pz1=*((t+j)->V[2]+2);
                        *pP1=ContourPoint
                             (getContourPoint
                              (*px0, *px1, *pz0, *pz1, z),
                              getContourPoint
                              (*py0, *py1, *pz0, *pz1, z));
                        //ClTemp.push_back(*pP1);
                        (m+j)->TriangleFlag=0;
                        OutAngle=1;
                        NextTriangle=(t+j)->N[OutAngle]/3;
                        InAngle=(t+j)->N[OutAngle]%3;
                        break;
            }

            /*Track the contour line among the triangles*/
            bool END=false;
            while(!END)
            {
                j=NextTriangle;
                ClTemp.push_back(*pP1);
                PointNum++;
                switch ((m+j)->TriangleFlag)
                {
                    //0-1, 0-2
                    case 3:
                            if(InAngle==1)
                            {
                                *px0=*((t+j)->V[0]+0);
                                *px1=*((t+j)->V[1]+0);
                                *py0=*((t+j)->V[0]+1);
                                *py1=*((t+j)->V[1]+1);
                                *pz0=*((t+j)->V[0]+2);
                                *pz1=*((t+j)->V[1]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(1, 2, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else if(InAngle==2)
                            {
                                *px0=*((t+j)->V[0]+0);
                                *px1=*((t+j)->V[2]+0);
                                *py0=*((t+j)->V[0]+1);
                                *py1=*((t+j)->V[2]+1);
                                *pz0=*((t+j)->V[0]+2);
                                *pz1=*((t+j)->V[2]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(1, 2, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else
                                std::cout<< "Inside Error!"<< std::endl;
                            break;
                    //0-1, 1-2
                    case 5: if(InAngle==0)
                            {
                                *px0=*((t+j)->V[0]+0);
                                *px1=*((t+j)->V[1]+0);
                                *py0=*((t+j)->V[0]+1);
                                *py1=*((t+j)->V[1]+1);
                                *pz0=*((t+j)->V[0]+2);
                                *pz1=*((t+j)->V[1]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(0, 2, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                             }
                            else if(InAngle==2)
                            {
                                *px0=*((t+j)->V[1]+0);
                                *px1=*((t+j)->V[2]+0);
                                *py0=*((t+j)->V[1]+1);
                                *py1=*((t+j)->V[2]+1);
                                *pz0=*((t+j)->V[1]+2);
                                *pz1=*((t+j)->V[2]+2);
                                *pP1=ContourPoint
                                    (getContourPoint
                                     (*px0, *px1, *pz0, *pz1, z),
                                     getContourPoint
                                     (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(0, 2, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else
                                std::cout<< "Inside Error!"<< std::endl;
                            break;
                    //1-2, 0-2
                    case 6: if(InAngle==0)
                            {
                                *px0=*((t+j)->V[0]+0);
                                *px1=*((t+j)->V[2]+0);
                                *py0=*((t+j)->V[0]+1);
                                *py1=*((t+j)->V[2]+1);
                                *pz0=*((t+j)->V[0]+2);
                                *pz1=*((t+j)->V[2]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(0, 1, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else if(InAngle==1)
                            {
                                *px0=*((t+j)->V[1]+0);
                                *px1=*((t+j)->V[2]+0);
                                *py0=*((t+j)->V[1]+1);
                                *py1=*((t+j)->V[2]+1);
                                *pz0=*((t+j)->V[1]+2);
                                *pz1=*((t+j)->V[2]+2);
                                *pP1=ContourPoint
                                     (getContourPoint
                                      (*px0, *px1, *pz0, *pz1, z),
                                      getContourPoint
                                      (*py0, *py1, *pz0, *pz1, z));
                                OutAngle=NotIn(0, 1, InAngle);
                                NextTriangle=(t+j)->N[OutAngle]/3;
                                InAngle=(t+j)->N[OutAngle]%3;
                                (m+j)->TriangleFlag=0;
                            }
                            else
                                std::cout<< "Inside Error!"<< std::endl;
                            break;
                    case 0: END=true;
                            break;
                    default:std::cout<<"Error occurs in the Inside Function"<<std::endl;
                            break;
                }
            }
            std::cout<< "The " << InsideContourNum
                     << " Inside Contour Line's Point Number Is : "
                     << PointNum << std::endl;
            ContourNum++;//move to the next contour line
            InsideContourNum++;
            cl.push_back(ClTemp);
        }
    }
    std::cout<<"Inside Contours' Number Is: "
             <<InsideContourNum<<std::endl;
    std::cout<<"sucess"<<std::endl;
    std::cout<<"All Contours' Number Is: "<<ContourNum<<std::endl;
    delete pP1;
    delete px0;
    delete px1;
    delete py0;
    delete py1;
    delete pz0;
    delete pz1;
    return InsideContourNum;
}

int NotIn(const int a, const int b, int c)
{
    if(a==b)
    {
        std::cout <<"Input error of NotIn Function" << std::endl;
        return 0;
    }
    else if(c==a)
    {
        return b;
    }
    else if(c==b)
    {
        return a;
    }
    else if(c!=a && c!=b)
    {
        std::cout << "Big error!!! This Program should be checked!"
                  << std::endl;
        return 4;
    }
    return 0;
}

void setTriangleFlag(TINtriangle *t1, Flag *m1, float z, long TinNum)
{
    float AcceptedError=0.00001f;
    for(long i=0; i<TinNum; i++)
    {
        Flag* m=m1+i;
        m->TriangleFlag=0;
        TINtriangle* t=t1+i;
        if(t->V[0])
        {
            if(fabs(z-*(t->V[0]+2))>AcceptedError && fabs(z-*(t->V[1]+2))>AcceptedError && (z-*(t->V[0]+2))*(z-*(t->V[1]+2))<0)
                m->TriangleFlag+=1;
            if(fabs(z-*(t->V[0]+2))>AcceptedError && fabs(z-*(t->V[2]+2))>AcceptedError && (z-*(t->V[0]+2))*(z-*(t->V[2]+2))<0)
                m->TriangleFlag+=2;
            if(fabs(z-*(t->V[1]+2))>AcceptedError && fabs(z-*(t->V[2]+2))>AcceptedError && (z-*(t->V[1]+2))*(z-*(t->V[2]+2))<0)
                m->TriangleFlag+=4;
            if(fabs(z-*(t->V[0]+2))<=AcceptedError)
                m->TriangleFlag+=10;
            if(fabs(z-*(t->V[1]+2))<=AcceptedError)
                m->TriangleFlag+=20;
            if(fabs(z-*(t->V[2]+2))<=AcceptedError)
                m->TriangleFlag+=40;
        }
        else
        {
            if(fabs(z-*(t->V[1]+2))>AcceptedError && fabs(z-*(t->V[2]+2))>AcceptedError && (z-*(t->V[1]+2))*(z-*(t->V[2]+2))<0)
                m->TriangleFlag+=100;
            if(fabs(z-*(t->V[1]+2))<=AcceptedError)
                m->TriangleFlag+=200;
            if(fabs(z-*(t->V[2]+2))<=AcceptedError)
                m->TriangleFlag+=400;
        }
    }
}

float getContourPoint(const float& x0, const float& x1, const float& z0, const float& z1, const float& z)
{
    std::decimal::decimal32 result, deltax, deltaz1, deltaz2, middleresult1,middleresult2;
    deltax=std::decimal::decimal32(x1-x0);
    deltaz1=std::decimal::decimal32(z-z0);
    deltaz2=std::decimal::decimal32(z1-z0);
    middleresult1=deltaz1/deltaz2;
    middleresult2=deltax*middleresult1;
    result=std::decimal::decimal32(x0)+middleresult2;
    return std::decimal::decimal32_to_float(result);
}
void ContourLine::filterNoise(int ThresholdPointNum)
{
    std::vector< std::vector<ContourPoint> >::iterator it;
    for (it=cl.begin(); it!=cl.end(); it++)
    {
        if(it->size()<=ThresholdPointNum)
        {
            it=cl.erase(it);
            it--;
        }
    }
}
void SlightChange(PointNode* p, long PointNum, float Zmin, float Zmax, const int n)
{
    float unit=(Zmax-Zmin)/n;
    float temp=Zmin;
    std::vector<float> points;
    for(int i=0; i<n; i++)
    {       
        points.push_back(temp);
        temp=temp+unit;
    }
    for(int j=0; j<PointNum; j++)
    {
        for(int i=0; i<n; i++)
        {
            if ((p+j)->xyz[2]==points[i])
            {
                (p+j)->xyz[2]+=0.001;
            }

        }
    }
}
void getAllContourLines(TINtriangle* t, Flag* f, Contour& c, float Zmin, float Zmax, const int n, long TinNum)
{
    if(n>=10)
    {
        std::cout<<"ContourLineFilterError"<<std::endl;
    }
    else
    {
        float unit=(Zmax-Zmin)/n;
        float temp=Zmin;
        ContourLine contourline;
        Properties p;
        for(int i=0; i<n; i++)
        {
            p=Properties(temp);
            setTriangleFlag(t, f, temp, TinNum);
            for(int j=0; j<TinNum; j++)
            {
                if (f[j].TriangleFlag!=3 &&
                    f[j].TriangleFlag!=5 &&
                    f[j].TriangleFlag!=6 &&
                    f[j].TriangleFlag!=100 &&
                    f[j].TriangleFlag!=0)
                    {
                        std::cout<<j<<std::endl;
                        std::cout<<f[j].TriangleFlag<<std::endl;
                    }
            }
            contourline.getContourLinefromBoundary(t, f, temp);
            contourline.getContourLinefromInside(t, f, temp);
            contourline.filterNoise(20);
            if(!contourline.cl.empty())
            {
                c.contour.insert(std::pair<Properties, ContourLine>(p, contourline));
            }
            temp=temp+unit;
            contourline.cl.clear();
        }
    }
}


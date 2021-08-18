#include "action.h"
using std::vector;

action::action()
{
    state1 = 1;
    state2 = 1;
    state3 = 1;
    state4 = 1;

    x1 = -0.025;
    y1 = 0;
    z1 = 0;
    x2 = 0.06;
    y2 = 0;
    z2 = 0;
    x3 = -0.025;
    y3 = 0;
    z3 = 0;
    x4 = 0.06;
    y4 = 0;
    z4 = 0;
}
vector<double> action::forward_walking(int *state, double *x, double *y, double *z, double *t, int num)
{
    double lengthy = 0.118;
    double lengthx = 0.190;
    double height = 0.15;
     vector<double> point1 = {0.06,0,0};
    vector<double> point2 = {-0.025,0,0};
    vector<double> point3 = {-0.035,0,0.01};

    
    if(*state == 1)
    {
        *x = *x + 1*(*t);
        *y = 0;
        *z = 0;
        if(*x >= point1[0] && z >= 0)
        {
            *state = 2;
            *t = 0;
        }
    }
    else if(*state == 3)
    {
        *x = *x + 1.5*(*t);
        *y = 0;
        *z = *z - 1.5*(*t);
        if(*x > point2[0] || *z <= 0)
        {
            *state = 1;
            *t = 0;
        }
    }
    else if(*state == 2)
    {
        *x = *x - (point1[0]-point3[0]) * 50 * (*t);
        *y = 0;
        *z = *z + (point3[2]*50)*(*t);
        if(*x <= point3[0] || *z >= point3[2])
        {
            *state = 3;
            *t = 0;
        }
    }

    vector<double> point;
    if(num == 1)
    {
        point.push_back(*x+lengthx/2);
        point.push_back(*y-lengthy/2);
        point.push_back(*z-height);
    }
    else if(num == 2)
    {
        point.push_back(*x+lengthx/2);
        point.push_back(*y+lengthy/2);
        point.push_back(*z-height);
    }
    else if(num == 3)
    {
        point.push_back(*x-lengthx/2);
        point.push_back(*y+lengthy/2);
        point.push_back(*z-height);
    }
    else if(num == 4)
    {
        point.push_back(*x-lengthx/2);
        point.push_back(*y-lengthy/2);
        point.push_back(*z-height);
    }
    
    return point;

}

vector<vector<double>> action::forward(double *t1, double *t2,double *t3,double *t4)
{
    vector<vector<double>> point = {forward_walking(&state1,&x1,&y1,&z1,t1,1),forward_walking(&state2,&x2,&y2,&z2,t2,2),
                                    forward_walking(&state3,&x3,&y3,&z3,t3,3),forward_walking(&state4,&x4,&y4,&z4,t4,4)};
    return point;
}
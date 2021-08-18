#ifndef __ACTION_H__
#define __ACTION_H__

#include <iostream>
#include <cmath>
#include <vector>
using std::vector;
class action
{
    public:
        int state1;
        int state2;
        int state3;
        int state4;
        double x1;
        double y1;
        double z1;
        double x2;
        double y2;      
        double z2;
        double x3;
        double y3;
        double z3;
        double x4;
        double y4;
        double z4;
    
        action();
        vector<double> forward_walking(int *state, double *x, double *y, double *z, double *t, int num);
        vector<vector<double>> forward(double *t1, double *t2,double *t3,double *t4);
};
#endif
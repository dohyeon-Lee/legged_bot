#ifndef __ACTION_H__
#define __ACTION_H__

#include <iostream>
#include <cmath>
#include <vector>
#include <unistd.h>
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

        vector<double> point1;
        vector<double> point2;

        //about PID
        double pre_P_x;
        double pre_P_y;

        double error_x;
        double error_y;

        double pre_error_x;
        double pre_error_y;
        action();
        vector<double> forward_walking(int *state, double *x, double *y, double *z, double *t, int num);
        vector<double> forward_walking_v2(int *state, double *x, double *y, double *z, double *t, int num);
        vector<vector<double>> forward(double *t1, double *t2,double *t3,double *t4);
        void groundslopePID_pre_setting();
        vector<double> groundslopePID(vector<double> goal, double angle_x, double angle_y);
        vector<double> walkingPID(vector<double> goal, double angle_x, double angle_y);
};
#endif
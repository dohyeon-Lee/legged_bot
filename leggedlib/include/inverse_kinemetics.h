#ifndef __INVERSE_KINEMETICS_H__
#define __INVERSE_KINEMETICS_H__

#include <iostream>
#include <cmath>
#include <vector>
class IK
{
    public:
        std::vector<double> IK_LF(double Yy, double Zz, double Xx);
        std::vector<double> IK_RF(double Yy, double Zz, double Xx);
        std::vector<double> IK_LB(double Yy, double Zz, double Xx);
        std::vector<double> IK_RB(double Yy, double Zz, double Xx);
        std::vector<std::vector<double>> inv_kinenmatics(double points[][3]);
        double dot(std::vector<double> point1, std::vector<double>point2);
        std::vector<std::vector<double>> plane(std::vector<double> normal, double l);
        std::vector<std::vector<double>> groundslope(std::vector<double> normal, double l);
        std::vector<double> sub(std::vector<double> v1, std::vector<double> v2);
        std::vector<double> add(std::vector<double> v1, std::vector<double> v2);
        std::vector<double> multiply(std::vector<double> v1, double v2);
};
#endif
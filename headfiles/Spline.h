#ifndef MANIPULATOR_SPLINE_H
#define MANIPULATOR_SPLINE_H

#include <iostream>
#include<cmath>
#include<queue>
#include<memory>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>

#include "TrajectoryPlanning.h"
#include "Discretization.h"



struct AfterCorrect{
    double ThetaOne;
    double ThetaTwo;
    double Zaixs;
    int status = -1;
};
struct AfterSplinePoint{
    std::queue<uint16_t> Theta_1;
    std::queue<uint16_t> Theta_2;
    std::queue<uint16_t> Z;
    std::queue<uint16_t> st;
};
struct Position{
    std::queue<AfterCorrect> LeftDown;
    std::queue<AfterCorrect> MiddleDown;
    std::queue<AfterCorrect> RightDown;
    std::queue<AfterCorrect> LeftUp;
    std::queue<AfterCorrect> RightUp;
    std::queue<AfterCorrect> MiddleUp;
    std::queue<AfterCorrect> PutBack;
};
struct Target{
    double x_final;
    double y_final;
    double z_final;
    int color;//red :1; 2:green; 3:blue
};


AfterCorrect Inverse(Trajectory_Point_t CorrectPoint, AfterCorrect Before);
Position Divide(const Trajectory_Point_t *CorrectPoint);
AfterSplinePoint Spline(std::queue<AfterCorrect> insert);
AfterSplinePoint Connection(int signal);

#endif //MANIPULATOR_SPLINE_H

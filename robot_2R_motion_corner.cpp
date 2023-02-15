// Markus Buchholz, 2023
// g++ robot_2R_motion_corner.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <tuple>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

/*
However, we usually avoid using arcsin and arccos because of the inaccuracy.
*/

//--------------------------------------------------------------------------------------------
float L1 = 1.0;
float L2 = 1.0;

//--------------------------------------------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>> FWD(std::vector<float> theta1, std::vector<float> theta2)
{

    std::vector<float> X;
    std::vector<float> Y;

    for (int ii = 0; ii < theta1.size(); ii++)
    {

        X.push_back(L1 * std::cos(theta1[ii]) + L2 * std::cos(theta1[ii] + theta2[ii]));
        Y.push_back(L1 * std::sin(theta1[ii]) + L2 * std::sin(theta1[ii] + theta2[ii]));
    }

    return make_tuple(X, Y);
}

//--------------------------------------------------------------------------------------------
std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> IK(std::vector<float> X, std::vector<float> Y)
{

    std::vector<float> theta11;
    std::vector<float> theta12;

    std::vector<float> theta21;
    std::vector<float> theta22;

    for (int ii = 0; ii < X.size(); ii++)
    {

        float a = std::sqrt(std::pow(L1 + L2, 2) - (X[ii] * X[ii] + Y[ii] * Y[ii]));
        float b = std::sqrt((X[ii] * X[ii] + Y[ii] * Y[ii]) - std::pow(L1 - L2, 2));

        // elbow up
        float t21 = 2 * std::atan2(a, b);
        // elbow down
        float t22 = -2 * std::atan2(a, b);

        theta21.push_back(t21);
        theta22.push_back(t22);

        // elbow up
        float t11 = std::atan2(Y[ii], X[ii]) + std::atan2(L2 * t21, L1 + L2 * std::cos(t21));
        // elbow down
        float t12 = std::atan2(Y[ii], X[ii]) - std::atan2(L2 * t22, L1 + L2 * std::cos(t22));

        theta11.push_back(t11);
        theta12.push_back(t12);
    }

    return std::make_tuple(theta11, theta21, theta12, theta22);
}
//---------------------------------------------------------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> generatePathAndCorners()
{

    float dt = 0.01;
    float tmax1 = 1.0;
    float tmax2 = 2.0;
    float tp = 0.1;

    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> time;

    for (float t = 0.0; t <= tmax1 - tp; t = t + dt)
    {

        Y.push_back(1.0 * t);
        X.push_back(1.0);
        time.push_back(t);
    }

    for (float t = tmax1 - tp + dt; t <= tmax1 + tp; t = t + dt)
    {

        float a = (1 - std::pow(t - 0.9, 2) / 0.4);
        float b = (1 - std::pow(t - 1.1, 2) / 0.4);
        X.push_back(a);
        Y.push_back(b);
        time.push_back(t);
    }

    for (float t = 1.0 + dt + tp; t <= tmax2; t = t + dt)
    {

        X.push_back(2.0 - t);
        Y.push_back(1.0);
        time.push_back(t);
    }

    return std::make_tuple(X, Y, time);
}
//---------------------------------------------------------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>> generatePath()
{


    float dt = 0.01;
    float tmax1 = 1.0;
    float tmax2 = 2.0;
    float tp = 0.1;

    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> time;

    for (float t = 0.0; t <= tmax1; t = t + dt)
    {

        Y.push_back(1.0 * t);
        X.push_back(1.0);
        time.push_back(t);
    }


    for (float t = 1.0 + dt; t <= tmax2; t = t + dt)
    {

        X.push_back(2.0 - t);
        Y.push_back(1.0);
        time.push_back(t);
    }

    return std::make_tuple(X, Y, time);



}
//---------------------------------------------------------------------------------------------------------

void plot2D(std::vector<float> t1, std::vector<float> t2, std::vector<float> time)
{
    plt::title("2R robot angles position. Linear motion. ");
    plt::named_plot("theta 2", time, t1);
    plt::named_plot("theta 2, corner-cutting", time, t2);
    plt::xlabel("time");
    plt::ylabel("angle");
    plt::legend();
    plt::xlabel("time");
    plt::ylabel("angle");
    plt::show();
}

//--------------------------------------------------------------------------------------------

void plotPath(std::vector<float> X, std::vector<float> Y, std::vector<float> Xcc, std::vector<float> Ycc)
{
    plt::title("2R robot path. ");
    plt::named_plot("robot path", X, Y);
    plt::named_plot("robot path, corner-cutting", Xcc, Ycc);
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::legend();
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::show();
}

//--------------------------------------------------------------------------------------------


int main()
{

    auto path = generatePath();
    auto pathCC = generatePathAndCorners();
    std::vector<float> time = std::get<2>(path);
    auto robotIK = IK(std::get<0>(path), std::get<2>(path));
    auto robotIKcc = IK(std::get<0>(pathCC), std::get<2>(pathCC));

    plot2D(std::get<0>(robotIK), std::get<0>(robotIKcc), time);
    plotPath(std::get<0>(pathCC), std::get<1>(path), std::get<0>(pathCC), std::get<1>(pathCC));

}
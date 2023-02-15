// Markus Buchholz, 2023
// g++ robot_2R_motion_corner.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>
#include <tuple>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

struct Point
{

    float x;
    float y;
    float z;
};

//---------------------------------------------------------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> generatePathAndCorners3D()
{

    float dt = 0.01;
    float t0 = 0.0;
    float t1 = 1.0;
    float t2 = 2.0;
    float tp = 0.2;

    Point p0 = {0, 0, 1};
    Point p1 = {1, 1, 1};
    Point p2 = {1, 1, 2};

    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> Z;
    std::vector<float> time;

    for (float t = t0; t <= t1 - tp; t = t + dt)
    {

        float x = p1.x - ((t1 - t) / (t1 - t0)) * (p1.x - p0.x);
        float y = p1.y - ((t1 - t) / (t1 - t0)) * (p1.y - p0.y);
        float z = p1.z - ((t1 - t) / (t1 - t0)) * (p1.z - p0.z);

        X.push_back(x);
        Y.push_back(y);
        Z.push_back(z);
        time.push_back(t);
    }

    for (float t = t1 - tp + dt; t <= t1 + tp; t = t + dt)
    {

        float x = p1.x - (std::pow(t - tp - t1, 2) / (4 * tp * (t1 - t0))) * (p1.x - p0.x) + (std::pow(t + tp - t1, 2) / (4 * tp * (t2 - t1))) * (p2.x - p1.x);
        float y = p1.y - (std::pow(t - tp - t1, 2) / (4 * tp * (t1 - t0))) * (p1.y - p0.y) + (std::pow(t + tp - t1, 2) / (4 * tp * (t2 - t1))) * (p2.y - p1.y);
        float z = p1.z - (std::pow(t - tp - t1, 2) / (4 * tp * (t1 - t0))) * (p1.z - p0.z) + (std::pow(t + tp - t1, 2) / (4 * tp * (t2 - t1))) * (p2.z - p1.z);

        X.push_back(x);
        Y.push_back(y);
        Z.push_back(z);
        time.push_back(t);
    }

    for (float t = t1 + dt + tp; t <= t2; t = t + dt)
    {

        float x = p1.x - ((t1 - t) / (t2 - t1)) * (p2.x - p1.x);
        float y = p1.y - ((t1 - t) / (t2 - t1)) * (p2.y - p1.y);
        float z = p1.z - ((t1 - t) / (t2 - t1)) * (p2.z - p1.z);

        X.push_back(x);
        Y.push_back(y);
        Z.push_back(z);
        time.push_back(t);
    }

    return std::make_tuple(X, Y, Z, time);
}
//---------------------------------------------------------------------------------------------------------

std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> generatePath3D()
{

    float dt = 0.01;
    float t0 = 0.0;
    float t1 = 1.0;
    float t2 = 2.0;

    Point p0 = {0, 0, 1};
    Point p1 = {1, 1, 1};
    Point p2 = {1, 1, 2};

    std::vector<float> X;
    std::vector<float> Y;
    std::vector<float> Z;
    std::vector<float> time;

    for (float t = t0; t <= t1; t = t + dt)
    {

        float x = p1.x - ((t1 - t) / (t1 - t0)) * (p1.x - p0.x);
        float y = p1.y - ((t1 - t) / (t1 - t0)) * (p1.y - p0.y);
        float z = p1.z - ((t1 - t) / (t1 - t0)) * (p1.z - p0.z);

        X.push_back(x);
        Y.push_back(y);
        Z.push_back(z);
        time.push_back(t);
    }

    for (float t = t1 + dt; t <= t2; t = t + dt)
    {

        float x = p1.x - ((t1 - t) / (t2 - t1)) * (p2.x - p1.x);
        float y = p1.y - ((t1 - t) / (t2 - t1)) * (p2.y - p1.y);
        float z = p1.z - ((t1 - t) / (t2 - t1)) * (p2.z - p1.z);

        X.push_back(x);
        Y.push_back(y);
        Z.push_back(z);
        time.push_back(t);
    }

    return std::make_tuple(X, Y, Z, time);
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

void plot3D(std::vector<float> xX, std::vector<float> yY, std::vector<float> zZ)
{

    plt::plot3(xX, yY, zZ);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("z");
    plt::show();
}

//---------------------------------------------------------------------------------------------------------
void plot3D3D(std::vector<float> xX, std::vector<float> yY, std::vector<float> zZ, std::vector<float> xXcc, std::vector<float> yYcc, std::vector<float> zZcc)
{

    plt::plot3(xXcc, yYcc, zZcc);
    plt::plot3(xX, yY, zZ);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("z");
    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("z");
    plt::show();
}
//---------------------------------------------------------------------------------------------------------

int main()
{

    auto path = generatePath3D();
    auto pathCC = generatePathAndCorners3D();
    std::vector<float> time = std::get<3>(pathCC);
    // plot3D(std::get<0>(pathCC), std::get<1>(pathCC), std::get<2>(pathCC));
    plot3D3D(std::get<0>(path), std::get<1>(path), std::get<2>(path), std::get<0>(pathCC), std::get<1>(pathCC), std::get<2>(pathCC));

    // plotPath(std::get<0>(pathCC), std::get<1>(path), std::get<0>(pathCC), std::get<1>(pathCC));
}
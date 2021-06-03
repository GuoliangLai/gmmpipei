//
// Created by lai on 2021/5/11.
//
#include <fstream>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
//#include <opencv2/xfeatures2d.hpp>        //SIFT SURF
#include <boost/math/distributions/normal.hpp>
#include <random>
#include <math.h>
#include <numeric>
#include <boost/make_shared.hpp>//boostÖžÕëÏà¹ØÍ·ÎÄŒþ
#define N 1
//#define vector2 std::vector<std::vector<double>>
using namespace std;
using namespace Eigen;
using namespace cv;
class Gridmap {
public:
    vector<double> u;
    vector<double> sigma;
    vector<double> Nk;
};//Define a grid class, which is used for dynamic arrays

class Gridcloud {
public:
    std::vector<double> pointz;
};//Define a grid class, which is used for dynamic arrays

class Gridgmm {
public:
    std::vector<double> gmmpro;
};//Define a grid class, which is used for dynamic arrays
struct point3d {
    float x;
    float y;
    float z;
};
void maxVector2d(std::vector<std::vector<double>> &arrays, int &maxIx, int &maxIy, double &maxElement);
int pipei(int argc, char **argv);
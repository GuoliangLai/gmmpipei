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
#include <boost/make_shared.hpp>
#include "pipei.h"
#define N 1

using namespace std;
using namespace Eigen;
using namespace cv;
/**
 * Grid
 */
//class Gridmap {
//public:
//    vector<double> u;
//    vector<double> sigma;
//    vector<double> Nk;6-1gmm
//};//Define a grid class, which is used for dynamic arrays

int compare(int argc, char **argv) {
    //read grid map
    FileStorage fs("6-1gmm.yaml", FileStorage::READ);
    float grid_size_x = fs["grid_size_x"];
    float grid_size_y = fs["grid_size_y"];
    int minx = fs["minx"];
    int maxx = fs["maxx"];
    int miny = fs["miny"];
    int maxy = fs["maxy"];
    int map_column = fs["column"];
    int map_row = fs["row"];
    cout << "grid_size_x：" << grid_size_x << endl;
    cout << "grid_size_y：" << grid_size_y << endl;
    cout << "minx：" << minx << endl;
    cout << "miny：" << miny << endl;
    cout << "maxx：" << maxx << endl;
    cout << "maxy：" << maxy << endl;
    cout << "column：" << map_column << endl;
    cout << "row：" << map_row << endl;

    //create the projection image of grid map
    cv::Mat mapimg = cv::Mat::zeros(map_row, map_column, CV_32FC1);;//64��

    //create dynamic arrays of grid map
    Gridmap** gridmap = new Gridmap*[map_row];
    for (int i = 0; i < map_row; ++i) {
        gridmap[i] = new Gridmap[map_column];
    }

    FileNode gmm = fs["gmm"];
    FileNodeIterator it = gmm.begin(), it_end = gmm.end();
    int idx = 0;

    FileStorage fs1("6-1gridgmm.yaml", FileStorage::WRITE);
    fs1 << "gridgmm" << "[";
    for (; it != it_end; ++it, idx++) {
        int colum_tmp, row_tmp;
        std::vector<double> a;

        colum_tmp = (int)(*it)["column"];
        row_tmp = (int)(*it)["row"];
        cout << colum_tmp << "," << row_tmp << endl;
        vector<float> gmm_utmp, gmm_sigmatmp, gmm_alphatmp;
        (*it)["gmmModeU"] >> gmm_utmp;
        (*it)["gmmModeSigma"] >> gmm_sigmatmp;
        (*it)["gmmModeAlpha"] >> gmm_alphatmp;

        fs1 << "{:" << "colum" << colum_tmp << "row" << row_tmp << "likelihood" << "[:";

        for (int i = 2; i <= 82; i++) {
            double p = 0;
            for (int j = 0; j < (int)gmm_alphatmp.size(); j++) {
                if (gmm_utmp[j]==0||
                gmm_sigmatmp[j]==0
                )
                    continue;
                else
                {
                    boost::math::normal_distribution<> norm((float)gmm_utmp[j], (float)gmm_sigmatmp[j]);
                    p = p + gmm_alphatmp[j] * pdf(norm, i / 10.0);
                }

            }
            double f;
            if (p == 0.0) {
                f = -800;
            }
            else {
                f = log(p);
            }
            fs1 << f;
        }

        fs1 << "]" << "}";
    }
    fs1 << "]";
    fs1.release();

    return 0;
}
float  computeGmmKl(Matrix<float,4,2> a,Matrix<float,4,2> b)
{


    float logpa=0,logpb=0;
    if (!a(1,1))
    {
        if (b(1,1))
        {
            return 244;
        }else
        {
            for (int i = 1; i < 150; ++i) {
                float pa,pb,z;
                z = ((float)i)/30.0;
                boost::math::normal_distribution<> norm1(a(0,0),a(1,0));
                boost::math::normal_distribution<> norm2(b(0,0),b(1,0));
                pa = pdf(norm1,z);
                pb = pdf(norm2,z);
                if (pa==0)
                    pa=1;
                if (pb==0)
                    pb=1;
                pa= log(pa);
                pb = log(pb);
                logpa+=pa;
                logpb+=pb;
            }

            logpa=logpa/150.0;
            logpb=logpb/150.0;
            return  logpa-logpb;
        }
    }else
    {
        if (!b(1,1))
        {
            return 233;
        }
        for (int i = 1; i < 150; ++i) {
            float pa,pb,z;
            z = ((float)i)/30.0;
            boost::math::normal_distribution<> norm11(a(0,0),a(1,0));
            boost::math::normal_distribution<> norm12(b(0,0),b(1,0));
            boost::math::normal_distribution<> norm21(a(0,0),a(1,0));
            boost::math::normal_distribution<> norm22(b(0,0),b(1,0));
            pa = a(2,0)*pdf(norm11,z)
                 +a(2,1)*pdf(norm12,z);
            pb = b(2,0)*pdf(norm21,z)
                 +b(2,1)*pdf(norm22,z);
            pa= log(pa);
            pb = log(pb);
            logpa+=pa;
            logpb+=pb;
        }

        logpa=logpa/150.0;
        logpb=logpb/150.0;
        return  logpa-logpb;
    }

}

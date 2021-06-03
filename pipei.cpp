//
// Created by lai on 2021/5/11.
//
#include <fstream>
#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/xfeatures2d.hpp>        //SIFT SURF
#include <boost/math/distributions/normal.hpp>
#include <random>
#include <math.h>
#include <numeric>
#include <boost/make_shared.hpp>
#define N 1
#include "pipei.h"
//#define vector2 std::vector<std::vector<double>>
using namespace std;
using namespace Eigen;
using namespace cv;
using namespace boost;
using namespace boost::tuples;

void maxVector2d(std::vector<std::vector<double>> &arrays, int &maxIx, int &maxIy, double &maxElement) {
    std::vector<std::vector<double>> a = arrays;
    double e = a[0][0];
    int ix = 0;
    int iy = 0;

    for (int i = 0; i < a.size(); i++) {
        for (int j = 0; j < a[0].size(); j++) {
            if (e < a[i][j]) {
                e = a[i][j];
                ix = i;
                iy = j;
            }
        }
    }
    maxIx = ix;
    maxIy = iy;
    maxElement = e;
}
/**
 * Grid
 */

int pipei(int argc, char **argv) {
    //read grid map
    FileStorage fs("cloudmapgmmgmm.yaml", FileStorage::READ);
    float grid_size_x = fs["grid_size_x"];
    float grid_size_y = fs["grid_size_y"];
    int minx = fs["minx"];
    int maxx = fs["maxx"];
    int miny = fs["miny"];
    int maxy = fs["maxy"];
    int map_column = fs["column"];
    int map_row = fs["row"];
    std::cout << "grid_size_x：" << grid_size_x << endl;
    std::cout << "grid_size_y：" << grid_size_y << endl;
    std::cout << "minx：" << minx << endl;
    std::cout << "miny：" << miny << endl;
    std::cout << "maxx：" << maxx << endl;
    std::cout << "maxy：" << maxy << endl;
    std::cout << "column：" << map_column << endl;
    std::cout << "row：" << map_row << endl;

    //create the projection image of grid map
    cv::Mat mapimg = cv::Mat::zeros(map_column, map_row, CV_32FC1);;//64��

    //create dynamic arrays of grid map
    Gridmap** gridmap = new Gridmap*[map_column];
    for (int i = 0; i < map_column; ++i) {
        gridmap[i] = new Gridmap[map_row];
    }

    FileNode gmm = fs["gmm"];
    FileNodeIterator it = gmm.begin(), it_end = gmm.end();

    for (; it != it_end; ++it) {
        int colum_tmp, row_tmp;
        std::vector<double> a;

        colum_tmp = (int)(*it)["column"];
        row_tmp = (int)(*it)["row"];

        (*it)["gmmMod"] >> a;

        int k = a.size() / 3;
        vector<float> gmm_utmp, gmm_sigmatmp, gmm_alphatmp;
        (*it)["gmmModeU"] >> gmm_utmp;
        (*it)["gmmModeSigma"] >> gmm_sigmatmp;
        (*it)["gmmModeAlpha"] >> gmm_alphatmp;
        for (int i = 0; i < (int) gmm_utmp.size(); ++i) {
            gridmap[colum_tmp][row_tmp].u.push_back(gmm_utmp[i]);
            gridmap[colum_tmp][row_tmp].sigma.push_back(gmm_sigmatmp[i]);
            gridmap[colum_tmp][row_tmp].Nk.push_back(gmm_alphatmp[i]);
        }

        mapimg.at<float>(colum_tmp, row_tmp) = 255;
        //cout<<"u1:"<<grid[colum_tmp][row_tmp].u1<<"sigma1:"<<grid[colum_tmp][row_tmp].sigma1
        //<<"Nk1:"<<grid[colum_tmp][row_tmp].Nk1<<"u2:"<<grid[colum_tmp][row_tmp].u2
        //<<"sigma2:"<<grid[colum_tmp][row_tmp].sigma2<<"Nk2:"<<grid[colum_tmp][row_tmp].Nk2<<endl;
    }
    fs.release();
    //imshow("mapimg", mapimg);
    imwrite("mapimg.jpg", mapimg);
    //waitKey(0);

    Gridgmm** gridgmm = new Gridgmm*[map_column];
    for (int i = 0; i < map_column; ++i) {
        gridgmm[i] = new Gridgmm[map_row];
    }

    FileStorage fs1("gridgmm.yaml", FileStorage::READ);
    FileNode grid_gmm = fs1["gridgmm"];
    FileNodeIterator it1 = grid_gmm.begin(), it1_end = grid_gmm.end();
    pcl::PointCloud<pcl::PointXYZ>::Ptr example(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("a.pcd", *example);
        for (; it1 != it1_end; ++it1) {
        int colum_tmp, row_tmp;
        vector<double> gmm_tmp;

        colum_tmp = (int)(*it1)["colum"];
        row_tmp = (int)(*it1)["row"];

        (*it1)["likelihood"] >> gmm_tmp;
        gridgmm[colum_tmp][row_tmp].gmmpro = gmm_tmp;
    }
    fs1.release();
    vector<point3d> cloudin;
    ifstream in("cloud.txt");//¶ÁÈ¡µØÍŒÎ»×Ë
    for (int i = 0; i < example->size(); ++i) {
        point3d point;
        point.x= example->points[i].x;
        point.y=example->points[i].y;
        point.z=example->points[i].z;
        cloudin.push_back(point);
    }

    int cloud_row = 25 / grid_size_x;
    int cloud_column = 25 / grid_size_y;
    Gridcloud** gridcloud = new Gridcloud*[cloud_column];
    for (int i = 0; i < cloud_column; ++i) {
        gridcloud[i] = new Gridcloud[cloud_row];
    }

    //create the projection image of grid map
    cv::Mat cloudimg = cv::Mat::zeros(cloud_row, cloud_column, CV_32FC1);;//64��

    for (int i = 0; i < cloudin.size(); ++i) {
        int colum_tmp, row_tmp;
        colum_tmp = (cloudin[i].x + 12.5) / grid_size_x;
        row_tmp = (cloudin[i].y + 12.5) / grid_size_y;
        if (colum_tmp >= 100 || row_tmp >= 100 || colum_tmp < 0 || row_tmp < 0)
            continue;
        else
        {
            double pz;
            pz = cloudin[i].z;
            //cout << pz << endl;
            if (pz < 0.2 || pz>8.2)
                continue;
            else
            {
                gridcloud[colum_tmp][row_tmp].pointz.push_back(pz);
                cloudimg.at<float>(colum_tmp, row_tmp) = 255;
            }
        }
    }

    //imshow("cloudimg", cloudimg);
    imwrite("cloudimg.jpg", cloudimg);
    //waitKey(0);

    //Set the start grid
    int x0, y0, x1, y1, x2, y2;//the location at t,t-1,t-2 times
//    x0 = 200;//int(ceil(g3[0].x - minx) / grid_size_x);
//    y0 = 372;//int(ceil(-g3[0].z - miny) / grid_size_y);
    x0 = 371;//int(ceil(g3[0].x - minx) / grid_size_x);
    y0 = 200;//int(ceil(-g3[0].z - miny) / grid_size_y);
    std::cout << "(x0,y0):" << x0 << "," << y0 << endl;
    //Sets the width of the extracted gridimg
    int xd = 12.5 / grid_size_x;
    int yd = 12.5 / grid_size_y;

    //Set state sequence
    std::vector<std::vector<int>> xm;//the x of the state grid
    std::vector<std::vector<int>> ym;//the y of the state grid
    std::vector<std::vector<cv::Mat>> map_img;//the matched img of the state grid

    for (int xn = 0; xn < 2 * N + 1; xn++) {
        vector<int> xm_tmp, ym_tmp;
        vector<Mat> mapimg_tmp;
        for (int yn = 0; yn < 2 * N + 1; yn++) {
            int xt, yt;

            xt = x0 + xn - N;
            yt = y0 + yn - N;

            xm_tmp.push_back(xt);
            ym_tmp.push_back(yt);

            Mat img_tmp(mapimg, Rect(xt - xd, yt - yd, 2 * xd, 2 * yd));
            //imshow("img_tmp", img_tmp);
            imwrite("img_tmp"+to_string(xt)+to_string(yt)+".jpg", img_tmp);
            //waitKey(0);

            mapimg_tmp.push_back(img_tmp);
        }
        xm.push_back(xm_tmp);
        ym.push_back(ym_tmp);
        map_img.push_back(mapimg_tmp);
    }

    //ofstream out("end.txt");
    boost::math::normal_distribution<> norm(0, 1);
    std::vector<std::vector<double>> gridpro;//the x of the state grid
    for (int xn = 0; xn < 2 * N + 1; xn++) {
        vector<double> pro_tmp1;
        for (int yn = 0; yn < 2 * N + 1; yn++) {
            std::cout << "(xn,yn):" << xn << yn << endl;
            double pro_tmp = 0;
            for (int i = 0; i < cloud_column; i++) {
                for (int j = 0; j < cloud_row; j++) {
                    //std::cout << "(row,col):" << i << "," << j << endl;
                    //out << "(row,col):" << i << "," << j << endl;
                    int xl = xm[xn][yn] - xd + i;
                    int yl = ym[xn][yn] - yd + j;
                    if (gridgmm[xl][yl].gmmpro.size() <= 0) {
                        if (gridcloud[i][j].pointz.size() == 0) {
                            continue;
                        }
                        else {
                            double p = pdf(norm, gridcloud[i][j].pointz.size());
                            if (p == 0.0) {
                                p = -800;
                            }
                            else {
                                p = log(p);
                            }

//                            cout << p << endl;
                            pro_tmp = pro_tmp + p;
                            //out << pro_tmp << endl;
                        }
                    }
                    else {
                        if (gridcloud[i][j].pointz.size() == 0) {
                            double p = -800;
                            //out << p << endl;
                            pro_tmp = pro_tmp + p;
                        }
                        else {
                            for (int n = 0; n < gridcloud[i][j].pointz.size(); n++) {
                                int zl = int((gridcloud[i][j].pointz[n] - 0.2) * 10);
                                //cout << zl << endl;
                                double p = gridgmm[xl][yl].gmmpro[zl];
                                //out << p << endl;
                                pro_tmp = pro_tmp + p;
                                //out << pro_tmp << endl;
                            }
                        }

                    }
                    //std::cout << pro_tmp << endl;
                }
            }
            std::cout << pro_tmp << endl;
            pro_tmp1.push_back(pro_tmp);
        }
        gridpro.push_back(pro_tmp1);
    }
    for (int i = 0; i < cloud_column; ++i) {
        delete[] gridgmm[i];
    }
    delete[] gridgmm;
    int maxIx, maxIy;
    double maxElement;
    maxVector2d(gridpro, maxIx, maxIy, maxElement);
    std::cout << "(x,y):" << maxIx << "," << maxIy << "," << maxElement;
    return 0;
}

//
// Created by leixiaoyu on 2022/4/1.
//

#ifndef VANISHINGPOINT_POSITION_CONTROLLER_H
#define VANISHINGPOINT_POSITION_CONTROLLER_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <Eigen/Dense>//包含Eigen矩阵运算库，用于矩阵计算
#include <cmath>
#include <limits>//用于生成随机分布数列
using namespace std;
using namespace cv;
class position_controller {
public:
    cv::Mat C,n,v;
    cv::Mat J_pdct,p,k,J_evlt,z_meas;
public:
    void getJ0(cv::Mat dy,cv::Mat dp);
    void KalmanFilter(vector<double> dy,vector<double> dp);
    position_controller();
};


#endif //VANISHINGPOINT_POSITION_CONTROLLER_H

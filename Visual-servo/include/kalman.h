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
using namespace std;
using namespace cv;
class kalman_controller {
private:
    cv::Mat C,n,v;
    cv::Mat J_pdct,p,k,z_meas;
public:
    cv::Mat J_evlt;
public:
    kalman_controller();
    void setJ0(cv::Mat dy,cv::Mat dp);
    void KalmanFilter(vector<double> dy,vector<double> dp);
};


#endif //VANISHINGPOINT_POSITION_CONTROLLER_H

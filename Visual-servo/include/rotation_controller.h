//
// Created by leixiaoyu on 2022/4/11.
//

#ifndef VANISHINGPOINT_ROTATION_CONTROLLER_H
#define VANISHINGPOINT_ROTATION_CONTROLLER_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
using namespace std;
using namespace cv;

class rotation_controller {
private:
    vector<cv::Mat> Hs;
    vector<cv::Mat> drs;
public:
    int size=4;
    cv::Mat J;
public:
    void add(cv::Mat dr,cv::Mat H);
    void setJ0(vector<cv::Mat> Hs_r,vector<cv::Mat> drs_r);
private:
    void getJ();
    cv::Mat kronecker(cv::Mat m1,cv::Mat m2);
};


#endif //VANISHINGPOINT_ROTATION_CONTROLLER_H

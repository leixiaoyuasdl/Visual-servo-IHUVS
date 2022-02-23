//
// Created by LeiXiaoYu on 2022/2/21.
//

#ifndef VISUAL_SERVO_FIRSTPROCESSOR_H
#define VISUAL_SERVO_FIRSTPROCESSOR_H
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace KDL;
using namespace cv;
struct data
{
    vector<int> sum;
    vector<double> x;
    vector<double> y;
    KDL::Vector v;
    KDL::Rotation r;

};
class firstprocessor {
public:
    vector<cv::Point2f> mvKeys;
    data da;
private:
    int map[100];
public:
    void init(data a);
    void GetFourPoints();
    void setmap();
private:
    std::vector<double> getline(int a0,int a1);
    int* GetVanishingPoint(std::vector<std::vector<double> > Lines);
    std::vector<std::vector<double>> FilterLines(const std::vector<std::vector<double> > &Lines);

};


#endif //VISUAL_SERVO_FIRSTPROCESSOR_H

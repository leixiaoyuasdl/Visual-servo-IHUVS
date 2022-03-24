//
// Created by LeiXiaoYu on 2022/2/21.
//

#ifndef VISUAL_SERVO_FIRSTPROCESSOR_H
#define VISUAL_SERVO_FIRSTPROCESSOR_H
#include "robot.h"
class firstprocessor {
public:
    vector<cv::Point2f> mvKeys;
    data da;
private:
    int map[100];
public:
    void init(data a);
    void GetFourPoints(Size PatSize1,int start);
    void setmap();
private:
    std::vector<double> getline(int a0,int a1);
    double* GetVanishingPoint(std::vector<std::vector<double> > Lines);
    std::vector<std::vector<double>> FilterLines(const std::vector<std::vector<double> > &Lines);

};


#endif //VISUAL_SERVO_FIRSTPROCESSOR_H

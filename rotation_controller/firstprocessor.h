//
// Created by LeiXiaoYu on 2022/2/21.
//

#ifndef VISUAL_SERVO_FIRSTPROCESSOR_H
#define VISUAL_SERVO_FIRSTPROCESSOR_H
#include "robot.h"
class firstprocessor {
public:
    vector<cv::Point2f> mvKeys;
    vector<cv::Point2f> allcorners;
    cv::Mat image;
    KDL::Frame frame;
public:
    void init(cv::Mat im,KDL::Frame f);

private:
    void GetFourPoints();
    void getchessboardcorners(Mat src,Size PatSize);
    std::vector<double> getline(int a0,int a1);
    double* GetVanishingPoint(std::vector<std::vector<double> > Lines);
    std::vector<std::vector<double>> FilterLines(const std::vector<std::vector<double> > &Lines);

};


#endif //VISUAL_SERVO_FIRSTPROCESSOR_H

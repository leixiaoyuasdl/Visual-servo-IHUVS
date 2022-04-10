//
// Created by lxy on 2022/2/21.
//

#ifndef VANISHINGPOINT_SECONDPROCESSOR_H
#define VANISHINGPOINT_SECONDPROCESSOR_H
#include "firstprocessor.h"

class secondprocessor {
private:

public:
    cv::Mat H;
    cv::Mat dr;
    firstprocessor fp1,fp2;
public:
    void ComputeH21();
    void ComputeH21_chu();
    void getdr();
    void init(firstprocessor &a,firstprocessor &b);
    void Normalize(const vector<cv::Point2f> &vKeys,
                   vector<cv::Point2f> &vNormalizedPoints,
                   cv::Mat &T);

};

#endif //VANISHINGPOINT_SECONDPROCESSOR_H

//
// Created by lxy on 2022/2/21.
//

#ifndef VANISHINGPOINT_THIRDPROCESSOR_H
#define VANISHINGPOINT_THIRDPROCESSOR_H

#include "secondprocessor.h"
class thirdprocessor {
public:
    vector<secondprocessor> sps;
    cv::Mat J_rotation;
public:
    void push_back(secondprocessor a);
    void getJ_rotation();
public:
    cv::Mat kronecker(cv::Mat m1,cv::Mat m2);
};

#endif //VANISHINGPOINT_THIRDPROCESSOR_H

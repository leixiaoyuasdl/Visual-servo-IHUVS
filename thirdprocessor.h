//
// Created by lxy on 2022/2/21.
//

#ifndef VANISHINGPOINT_THIRDPROCESSOR_H
#define VANISHINGPOINT_THIRDPROCESSOR_H

#include "secondprocessor.h"
class thirdprocessor {
private:
    vector<secondprocessor> sps;
    cv::Mat J_rotation;
public:
    void push_back(secondprocessor a);
    cv::Mat kronecker(cv::Mat m1,cv::Mat m2);
    void getJ_rotation();
};

#endif //VANISHINGPOINT_THIRDPROCESSOR_H

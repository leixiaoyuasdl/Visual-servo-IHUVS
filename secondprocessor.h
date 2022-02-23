//
// Created by lxy on 2022/2/21.
//

#ifndef VANISHINGPOINT_SECONDPROCESSOR_H
#define VANISHINGPOINT_SECONDPROCESSOR_H
#include "firstprocessor.h"

class secondprocessor {
private:
    firstprocessor fp1,fp2;
public:
    cv::Mat H;
    cv::Mat dr;
public:
    void ComputeH21();
    void getdr();
    void init(firstprocessor a,firstprocessor b);

};

#endif //VANISHINGPOINT_SECONDPROCESSOR_H

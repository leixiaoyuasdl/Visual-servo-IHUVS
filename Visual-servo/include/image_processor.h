#ifndef VANISHINGPOINT_IMAGE_PROCESSOR_H
#define VANISHINGPOINT_IMAGE_PROCESSOR_H
#include <iostream>
#include <fstream>
#include <vector>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace KDL;
using namespace cv;

class image_processor {
private:
    vector<cv::Point2f> allcorners;
public:
    Mat draw;
public:
    vector<cv::Point2f> getallcorners(Mat img);
    void getchessboardcorners(Mat src,Size PatSize);
    void Normalize(const vector<cv::Point2f> &vKeys,
                   vector<cv::Point2f> &vNormalizedPoints,
                   cv::Mat &T);
    cv::Mat ComputeH21(cv::Mat img1,cv::Mat img2);
};


#endif //VANISHINGPOINT_IMAGE_PROCESSOR_H

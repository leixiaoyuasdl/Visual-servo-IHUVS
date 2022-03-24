//
// Created by lxy on 2022/2/28.
//

#ifndef VANISHINGPOINT_ROBOT_H
#define VANISHINGPOINT_ROBOT_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include "CornerDetAC.h"
#include "ChessboradStruct.h"


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
class robot {
private:
    data da;
    std::string refFrame;
    std::string childFrame;
public:
    robot(std::string reF,std::string chF);
    bool getData(data &a);
private:
    bool getTransform();
    bool getFeatures();
    void getchessboardcorners(Mat src,Size PatSize);
};


#endif //VANISHINGPOINT_ROBOT_H

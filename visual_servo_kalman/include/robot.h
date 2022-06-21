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
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;
using namespace KDL;
using namespace cv;

class robot {
private:
    std::string refFrame;
    std::string childFrame;
public:
    cv::Mat image;
    KDL::Frame frame;
public:
    robot();
    bool getData();
    void move_robots(moveit::planning_interface::MoveGroupInterface *arm,geometry_msgs::Pose target_pose);
private:
    bool getTransform();
    bool getPic();

};


#endif //VANISHINGPOINT_ROBOT_H

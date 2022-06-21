//
// Created by leixiaoyu on 2022/4/9.
//

#ifndef VANISHINGPOINT_CONTROLLER_H
#define VANISHINGPOINT_CONTROLLER_H
#include "kalman.h"
#include "robot.h"
#include "image_processor.h"
class controller_kalman_6dof {
public:
    double k=0.3;
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
    Mat img_goal;
    vector<cv::Point2f> allcorners_goal;
    image_processor im_p;
    robot rob;
private:
    geometry_msgs::Pose target_pose;
    vector<double> target_joints;
    kalman_controller pos_con;

public:
    void position_control();
    void position_init();
};


#endif //VANISHINGPOINT_CONTROLLER_H

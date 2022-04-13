//
// Created by leixiaoyu on 2022/4/9.
//

#ifndef VANISHINGPOINT_CONTROLLER_H
#define VANISHINGPOINT_CONTROLLER_H
#include "position_controller.h"
#include "rotation_controller.h"
#include "robot.h"
#include "image_processor.h"
class controller {
public:
    double k=1;
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
    Mat img_goal;
    vector<cv::Point2f> allcorners_goal;
    image_processor im_p;
    robot rob;
private:
    geometry_msgs::Pose target_pose;
    position_controller pos_con;
    rotation_controller rot_con;

public:
    void rotation_control(); //sign=1 chu pi pei ;sign=0 jing pi pei
    void position_control();
    void rotation_init();
    void position_init();
};


#endif //VANISHINGPOINT_CONTROLLER_H

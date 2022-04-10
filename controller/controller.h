//
// Created by leixiaoyu on 2022/4/9.
//

#ifndef VANISHINGPOINT_CONTROLLER_H
#define VANISHINGPOINT_CONTROLLER_H
#include "thirdprocessor.h"
#include "position_controller.h"


class controller {
public:
    geometry_msgs::Pose target_pose;
    firstprocessor fp_goal;
    robot rob;
    boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
    thirdprocessor tp;
    position_controller pos_con;
    double k=0.5;
    firstprocessor fp;
    firstprocessor fp_pre;
public:
    void rotation_control(int sign); //sign=1 chu pi pei ;sign=0 jing pi pei
    void position_control();
    void rotation_init();
    void position_init();
};


#endif //VANISHINGPOINT_CONTROLLER_H

//
// Created by leixiaoyu on 2022/4/9.
//

#include "controller_kalman_6dof.h"

void controller_kalman_6dof::position_control()
{
    vector<double> joints;
    vector<cv::Point2f> allcorners;
    rob.getData();
    allcorners = im_p.getallcorners(rob.image);
    target_joints = arm->getCurrentJointValues();
    joints = arm->getCurrentJointValues();

    cv::Mat dy=cv::Mat(6,1,CV_64F);
    dy.at<double>(0)=allcorners_goal.at(0).x-allcorners.at(0).x;
    dy.at<double>(1)=allcorners_goal.at(0).y-allcorners.at(0).y;
    dy.at<double>(2)=allcorners_goal.at(29).x-allcorners.at(29).x;
    dy.at<double>(3)=allcorners_goal.at(29).y-allcorners.at(29).y;
    dy.at<double>(4)=allcorners_goal.at(30).x-allcorners.at(30).x;
    dy.at<double>(5)=allcorners_goal.at(30).y-allcorners.at(30).y;


    cv::Mat dp=cv::Mat(6,1,CV_64F);
    dp = pos_con.J_evlt.reshape(0,6).inv()*dy;
    cout<<"dp"<<endl<<dp<<endl;

    for(int i=0;i<6;i++)
    {
        target_joints[i] = target_joints[i] +k*(dp.at<double>(i));
    }

    arm->setJointValueTarget(target_joints);
    arm->move();

    vector<Point2f> allcorners2;
    rob.getData();
    allcorners2 = im_p.getallcorners(rob.image);

    vector<double> dy_v;
    vector<double> dp_v;

    dy_v.push_back(allcorners2.at(0).x-allcorners.at(0).x);
    dy_v.push_back(allcorners2.at(0).y-allcorners.at(0).y);
//    dy_v.push_back(fp.allcorners.at(30).x-fp_pre.allcorners.at(30).x);
    dy_v.push_back(allcorners2.at(29).x-allcorners.at(29).x);
    dy_v.push_back(allcorners2.at(29).y-allcorners.at(29).y);
    dy_v.push_back(allcorners2.at(30).x-allcorners.at(30).x);
    dy_v.push_back(allcorners2.at(30).y-allcorners.at(30).y);

    target_joints = arm->getCurrentJointValues();

    for(int i=0;i<6;i++)
        dp_v.push_back(target_joints[i]-joints[i]);

    pos_con.KalmanFilter(dy_v,dp_v);

}

void controller_kalman_6dof::position_init()
{
    cv::Mat dys=cv::Mat(6,6,CV_64F);
    cv::Mat dps=cv::Mat(6,6,CV_64F);
    int sum_pos =7;
    vector<double> joints;
    vector<cv::Point2f> allcorners;
    for(int i=0;i<sum_pos;i++)
    {
        rob.getData();
        if(i==0)
        {
            allcorners = im_p.getallcorners(rob.image);
            joints = arm->getCurrentJointValues();
        }
        else
        {
            vector<Point2f> allcorners2;
            vector<double> joints2;
            allcorners2 = im_p.getallcorners(rob.image);
            joints2 = arm->getCurrentJointValues();
            for(int j=0;j<6;j++)
            {
                dps.at<double>((i-1)*6+j)=joints2[j]-joints[j];
            }

            dys.at<double>((i-1)*6)=allcorners2.at(0).x-allcorners.at(0).x;
            dys.at<double>((i-1)*6+1)=allcorners2.at(0).y-allcorners.at(0).y;
            dys.at<double>((i-1)*6+2)=allcorners2.at(29).x-allcorners.at(29).x;
            dys.at<double>((i-1)*6+3)=allcorners2.at(29).y-allcorners.at(29).y;
            dys.at<double>((i-1)*6+4)=allcorners2.at(30).x-allcorners.at(30).x;
            dys.at<double>((i-1)*6+5)=allcorners2.at(30).y-allcorners.at(30).y;

            allcorners = allcorners2;
            joints  = joints2 ;
        }
        if(i==sum_pos-1) break;

        target_joints= arm->getCurrentJointValues();

        target_joints[i] = target_joints[i] - 0.01;

        arm->setJointValueTarget(target_joints);
        arm->move();
    }
    dps = dps.t();
    dys = dys.t();

    pos_con.setJ0(dys,dps);
}
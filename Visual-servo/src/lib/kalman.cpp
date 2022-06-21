//
// Created by leixiaoyu on 2022/4/1.
//

#include "kalman.h"

kalman_controller::kalman_controller()
{
p=cv::Mat::eye(6*6,6*6,CV_64F);
n=cv::Mat::eye(6*6,6*6,CV_64F);
v=0.1*cv::Mat::eye(6,6,CV_64F);
}

void kalman_controller::setJ0(cv::Mat dys, cv::Mat dps)
{
    J_evlt = dys*dps.inv();
    int sum;
    sum = J_evlt.cols*J_evlt.rows;
    J_evlt  = J_evlt.reshape(0, sum);
}

void kalman_controller::KalmanFilter(vector<double> dy, vector<double> dp)
{
    z_meas = cv::Mat::zeros(dy.size(),1,CV_64F);
    C = cv::Mat::zeros(dy.size(),dy.size()*dp.size(),CV_64F);
    for(int i=0;i<dy.size();i++)
    {
        z_meas.at<double>(i) = dy.at(i);
    }

    cv::Mat p_xyz=cv::Mat(1,6,CV_64F);
    for(int i=0;i<6;i++)
    {
        p_xyz.at<double>(i) = dp.at(i);
    }


    for(int i=0;i<dp.size();i++)
    {
        p_xyz.copyTo(C(Rect(6*i,i,6,1))) ;
    }

    J_pdct = J_evlt ;

    //预测状态与真实状态的协方差矩阵，P
    p = p  + n;
    //K:2x1
    cv::Mat tmp;
    tmp = C * p * C.t() + v;
    k = p* C.t() * tmp.inv();

    //估计值
    J_evlt = J_pdct + k * (z_meas - C * J_pdct);

    //估计状态和真实状态的协方差矩阵，P
    p = (Mat::eye(6*6,6*6,CV_64F) - k * C) * p;
}


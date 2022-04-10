//
// Created by leixiaoyu on 2022/4/9.
//

#include "controller.h"
void controller::rotation_control(int sign)
{
    tp.getJ_rotation();

    target_pose= arm->getCurrentPose().pose;

    cv::Mat R_real,R_sim,H_sim;
    secondprocessor test;
    test.init(fp,fp_goal);
    if(sign==0)
        test.ComputeH21();
    else
        test.ComputeH21_chu();
    test.getdr();
    H_sim=test.H;
    R_sim = tp.J_rotation.inv()*H_sim*tp.J_rotation;

    cv::Mat u,aw,vt;

    SVD::compute(R_sim,aw,u,vt);

    R_sim = u*vt;

    KDL::Rotation r(R_sim.at<double>(0),R_sim.at<double>(1),R_sim.at<double>(2),
                    R_sim.at<double>(3),R_sim.at<double>(4),R_sim.at<double>(5),
                    R_sim.at<double>(6),R_sim.at<double>(7),R_sim.at<double>(8));

    KDL::Rotation goal_r;
    goal_r = fp.frame.M*r;

    double x,y,z,w;
    goal_r.GetQuaternion(x,y,z,w);
    target_pose.orientation.x=x;
    target_pose.orientation.y=y;
    target_pose.orientation.z=z;
    target_pose.orientation.w=w;

    rob.move_robots(arm.get(),target_pose);

    rob.getData();
    fp_pre.init(fp.image,fp.frame);
    fp.init(rob.image,rob.frame);

    secondprocessor sp;
    sp.init(fp,fp_pre);
    sp.getdr();
    if(sign==0)
        sp.ComputeH21();
    else
        sp.ComputeH21_chu();
    tp.push_back(sp);

    if(tp.sps.size() > 4 )
    {
        vector<secondprocessor>::iterator kk = tp.sps.begin();
        tp.sps.erase(kk);
    }
}

void controller::position_control()
{
    target_pose= arm->getCurrentPose().pose;

    cv::Mat dy=cv::Mat(3,1,CV_64F);
    dy.at<double>(0)=fp_goal.allcorners.at(0).x-fp.allcorners.at(0).x;
    dy.at<double>(1)=fp_goal.allcorners.at(0).y-fp.allcorners.at(0).y;
    //dy.at<double>(2)=fp_goal.allcorners.at(30).x-fp.allcorners.at(30).x;
    dy.at<double>(2)=fp_goal.allcorners.at(10).x-fp.allcorners.at(10).x;

    cv::Mat dp=cv::Mat(3,1,CV_64F);
    dp = pos_con.J_evlt.reshape(0,3).inv()*dy;

//        cout<<"J"<<endl<<pos_con.J_evlt.reshape(0,3)<<endl;

    target_pose.position.x=target_pose.position.x+k*dp.at<double>(0);
    target_pose.position.y=target_pose.position.y+k*dp.at<double>(1);
    target_pose.position.z=target_pose.position.z+k*dp.at<double>(2);

    rob.move_robots(arm.get(),target_pose);

    rob.getData();

    fp_pre.init(fp.image, fp.frame);
    fp.init(rob.image, rob.frame);

    vector<double> dy_v;
    vector<double> dp_v;

    dy_v.push_back(fp.allcorners.at(0).x-fp_pre.allcorners.at(0).x);
    dy_v.push_back(fp.allcorners.at(0).y-fp_pre.allcorners.at(0).y);
//    dy_v.push_back(fp.allcorners.at(30).x-fp_pre.allcorners.at(30).x);
    dy_v.push_back(fp.allcorners.at(10).x-fp_pre.allcorners.at(10).x);

    dp_v.push_back(fp.frame.p.x()-fp_pre.frame.p.x());
    dp_v.push_back(fp.frame.p.y()-fp_pre.frame.p.y());
    dp_v.push_back(fp.frame.p.z()-fp_pre.frame.p.z());

    pos_con.KalmanFilter(dy_v,dp_v);
}

void controller::rotation_init()
{
    KDL::Rotation r[4] ;
    r[0] = Rotation::RotX(0.03);
    r[1] = Rotation::RotY(0.03);
    r[2] = Rotation::RotZ(0.03);
    int sum_rot =4;
    for(int i=0;i<sum_rot;i++)
    {
        rob.getData();

        if(i==0)
        {
            fp.init(rob.image,rob.frame);
        }

        else
        {
            fp_pre.init(fp.image,fp.frame);
            fp.init(rob.image,rob.frame);

            secondprocessor sp;
            sp.init(fp_pre,fp);
            sp.getdr();
            sp.ComputeH21();

            tp.push_back(sp);
        }

        if(i==sum_rot-1) break;

        target_pose= arm->getCurrentPose().pose;
        Rotation rx=Rotation::Quaternion(target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);
        KDL::Rotation goal_r;
        goal_r = rx*r[i];

        double x,y,z,w;

        goal_r.GetQuaternion(x,y,z,w);
        target_pose.orientation.x=x;
        target_pose.orientation.y=y;
        target_pose.orientation.z=z;
        target_pose.orientation.w=w;

        rob.move_robots(arm.get(),target_pose);
    }
}

void controller::position_init()
{
    cv::Mat dys=cv::Mat(3,3,CV_64F);
    cv::Mat dps=cv::Mat(3,3,CV_64F);
    int sum_pos =4;
    for(int i=0;i<sum_pos;i++)
    {
        rob.getData();
        if(i==0)
        {
            fp.init(rob.image,rob.frame);
        }
        else {
            fp_pre.init(fp.image, fp.frame);
            fp.init(rob.image, rob.frame);

            dps.at<double>((i-1)*3)=fp.frame.p.x()-fp_pre.frame.p.x();
            dps.at<double>((i-1)*3+1)=fp.frame.p.y()-fp_pre.frame.p.y();
            dps.at<double>((i-1)*3+2)=fp.frame.p.z()-fp_pre.frame.p.z();

            dys.at<double>((i-1)*3)=fp.allcorners.at(0).x-fp_pre.allcorners.at(0).x;
            dys.at<double>((i-1)*3+1)=fp.allcorners.at(0).y-fp_pre.allcorners.at(0).y;
//            dys.at<double>((i-1)*3+2)=fp.allcorners.at(30).x-fp_pre.allcorners.at(30).x;
            dys.at<double>((i-1)*3+2)=fp.allcorners.at(10).x-fp_pre.allcorners.at(10).x;
        }
        if(i==sum_pos-1) break;

        target_pose= arm->getCurrentPose().pose;

        if(i==0)
            target_pose.position.x = target_pose.position.x+0.03;
        if(i==1)
            target_pose.position.y = target_pose.position.y+0.03;
        if(i==2)
            target_pose.position.z = target_pose.position.z+0.03;

        rob.move_robots(arm.get(),target_pose);
    }
    dps = dps.t();
    dys = dys.t();

    pos_con.getJ0(dys,dps);
}
//
// Created by leixiaoyu on 2022/4/9.
//

#include "controller.h"
void controller::rotation_control()
{
//    tp.getJ_rotation();

    cv::Mat im;
    Frame F;
    rob.getData();
    rob.image.copyTo(im);
    F = rob.frame;

    target_pose= arm->getCurrentPose().pose;

    cv::Mat R_sim,H_sim;

    H_sim = im_p.ComputeH21(im,img_goal);

    R_sim = rot_con.J.inv()*H_sim*rot_con.J;

    cv::Mat u,aw,vt;

    SVD::compute(R_sim,aw,u,vt);

    R_sim = u*vt;

    KDL::Rotation r(R_sim.at<double>(0),R_sim.at<double>(1),R_sim.at<double>(2),
                    R_sim.at<double>(3),R_sim.at<double>(4),R_sim.at<double>(5),
                    R_sim.at<double>(6),R_sim.at<double>(7),R_sim.at<double>(8));

    KDL::Rotation goal_r;
    goal_r = rob.frame.M*r;

    double x,y,z,w;
    goal_r.GetQuaternion(x,y,z,w);
    target_pose.orientation.x=x;
    target_pose.orientation.y=y;
    target_pose.orientation.z=z;
    target_pose.orientation.w=w;

    rob.move_robots(arm.get(),target_pose);

    rob.getData();

    Mat dr,dH;

    dH = im_p.ComputeH21(rob.image,im);

    KDL::Rotation rr;
    rr = rob.frame.M.Inverse()*F.M;
    cv::Mat a(3,3,CV_64F,rr.data);
    a.copyTo(dr);
    rot_con.add(dr,dH);

//    cout<<"H"<<endl<<sp.H<<endl;
//    rot_con.KalmanFilter(sp.H,sp.dr);
}

void controller::position_control()
{
    Frame  F;
    vector<cv::Point2f> allcorners;
    rob.getData();
    allcorners = im_p.getallcorners(rob.image);
    F = rob.frame;

    target_pose= arm->getCurrentPose().pose;

    cv::Mat dy=cv::Mat(3,1,CV_64F);
    dy.at<double>(0)=allcorners_goal.at(0).x-allcorners.at(0).x;
    dy.at<double>(1)=allcorners_goal.at(0).y-allcorners.at(0).y;
    //dy.at<double>(2)=fp_goal.allcorners.at(30).x-fp.allcorners.at(30).x;
    dy.at<double>(2)=allcorners_goal.at(10).x-allcorners.at(10).x;

    cv::Mat dp=cv::Mat(3,1,CV_64F);
    dp = pos_con.J_evlt.reshape(0,3).inv()*dy;

//        cout<<"J"<<endl<<pos_con.J_evlt.reshape(0,3)<<endl;

    target_pose.position.x=target_pose.position.x+k*dp.at<double>(0);
    target_pose.position.y=target_pose.position.y+k*dp.at<double>(1);
    target_pose.position.z=target_pose.position.z+k*dp.at<double>(2);

    rob.move_robots(arm.get(),target_pose);

    vector<Point2f> allcorners2;
    rob.getData();
    allcorners2 = im_p.getallcorners(rob.image);

    vector<double> dy_v;
    vector<double> dp_v;

    dy_v.push_back(allcorners2.at(0).x-allcorners.at(0).x);
    dy_v.push_back(allcorners2.at(0).y-allcorners.at(0).y);
//    dy_v.push_back(fp.allcorners.at(30).x-fp_pre.allcorners.at(30).x);
    dy_v.push_back(allcorners2.at(10).x-allcorners.at(10).x);

    dp_v.push_back(rob.frame.p.x()-F.p.x());
    dp_v.push_back(rob.frame.p.y()-F.p.y());
    dp_v.push_back(rob.frame.p.z()-F.p.z());

    pos_con.KalmanFilter(dy_v,dp_v);

}

void controller::rotation_init()
{
    KDL::Rotation r[4] ;
    r[0] = Rotation::RotX(0.03);
    r[1] = Rotation::RotY(0.03);
    r[2] = Rotation::RotZ(0.03);
    int sum_rot =4;

    vector<cv::Mat> Hs;
    vector<cv::Mat> drs;

    Mat img;
    Frame F;
    for(int i=0;i<sum_rot;i++)
    {
        rob.getData();

        if(i==0)
        {
            rob.image.copyTo(img);
            F = rob.frame;
        }

        else
        {
            Mat dH,dr;
            dH=im_p.ComputeH21(img,rob.image);

            KDL::Rotation rr;
            rr = F.M.Inverse()*rob.frame.M;

            Hs.push_back(dH);
            cv::Mat a(3,3,CV_64F,rr.data);
            a.copyTo(dr);
            drs.push_back(dr);

            rob.image.copyTo(img);
            F = rob.frame;
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

    rot_con.setJ0(Hs,drs);
}

void controller::position_init()
{
    cv::Mat dys=cv::Mat(3,3,CV_64F);
    cv::Mat dps=cv::Mat(3,3,CV_64F);
    int sum_pos =4;
    Frame F;
    vector<cv::Point2f> allcorners;
    for(int i=0;i<sum_pos;i++)
    {
        rob.getData();
        if(i==0)
        {
            allcorners = im_p.getallcorners(rob.image);
            F = rob.frame;
        }
        else
        {
            vector<Point2f> allcorners2;
            allcorners2 = im_p.getallcorners(rob.image);

            dps.at<double>((i-1)*3)=rob.frame.p.x()-F.p.x();
            dps.at<double>((i-1)*3+1)=rob.frame.p.y()-F.p.y();
            dps.at<double>((i-1)*3+2)=rob.frame.p.z()-F.p.z();

            dys.at<double>((i-1)*3)=allcorners2.at(0).x-allcorners.at(0).x;
            dys.at<double>((i-1)*3+1)=allcorners2.at(0).y-allcorners.at(0).y;
//            dys.at<double>((i-1)*3+2)=fp.allcorners.at(30).x-fp_pre.allcorners.at(30).x;
            dys.at<double>((i-1)*3+2)=allcorners2.at(10).x-allcorners.at(10).x;

            allcorners = allcorners2;
            F = rob.frame;
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

    pos_con.setJ0(dys,dps);
}
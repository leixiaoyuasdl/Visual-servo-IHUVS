//
// Created by lxy on 2022/2/21.
//
#include "firstprocessor.h"
#include "position_controller.h"
int main(int argc,char **argv)
{

    ros::init(argc, argv, "lunwen");
    ros::AsyncSpinner spinner(5);
    spinner.start();


    robot rob;
    vector<cv::Mat> images;
    vector<KDL::Frame> frames;


    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    geometry_msgs::Pose target_pose;


//    if(rob.getData()==false) return 1;
//    std::string path = "/home/leixiaoyu/桌面/pic/goal.jpg";
//    cv::imwrite(path.c_str(), rob.image);
//
//    char a;
//    cin>>a;

    std::string path = "/home/leixiaoyu/桌面/pic/goal.jpg";

    images.push_back(imread(path.c_str() , -1));
    if(rob.getData()==false) return 1;
    frames.push_back(rob.frame);

    int sum =4;
    for(int i=0;i<sum;i++)
    {
//        char y;
//        while (1)
        {
//            cout<<"is ok?"<<endl;
//            cin>>y;
//            if(y=='y')
            {
                if(rob.getData()==false) return 1;
                images.push_back(rob.image);
                frames.push_back(rob.frame);

                if(i==sum-1) break;

                target_pose= arm.getCurrentPose().pose;

                if(i==0)
                    target_pose.position.x = target_pose.position.x+0.03;
                if(i==1)
                    target_pose.position.y = target_pose.position.y+0.03;
                if(i==2)
                    target_pose.position.z = target_pose.position.z+0.03;


                rob.move_robots(arm,target_pose);

//                break;
            }
        }
    }

    ofstream myfile("/home/leixiaoyu/桌面/data/r_data/data_result_pos.txt");
    if (!myfile.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }

    firstprocessor fp[1000];
    for(int i=0;i<5;i++)
    {
        fp[i].init(images.at(i),frames.at(i));
    }

    myfile<<fp[0].allcorners.at(0)<<" "<<fp[0].allcorners.at(29)<<" "<<fp[0].allcorners.at(30)<<" "<<fp[0].allcorners.at(49)<<endl;
    myfile.clear();


    cv::Mat dys=cv::Mat(3,3,CV_64F);
    cv::Mat dps=cv::Mat(3,3,CV_64F);


    for(int i=0;i<3;i++)
    {

        dps.at<double>(i*3)=fp[i+2].frame.p.x()-fp[i+1].frame.p.x();
        dps.at<double>(i*3+1)=fp[i+2].frame.p.y()-fp[i+1].frame.p.y();
        dps.at<double>(i*3+2)=fp[i+2].frame.p.z()-fp[i+1].frame.p.z();

        dys.at<double>(i*3)=fp[i+2].allcorners.at(0).x-fp[i+1].allcorners.at(0).x;
        dys.at<double>(i*3+1)=fp[i+2].allcorners.at(0).y-fp[i+1].allcorners.at(0).y;
        dys.at<double>(i*3+2)=fp[i+2].allcorners.at(30).x-fp[i+1].allcorners.at(30).x;
    }


    dps = dps.t();
    dys = dys.t();

    position_controller pos_con;
    pos_con.getJ0(dys,dps);


    int sum_diedai=1000;
    double r=1;
    for(int i=0;i<sum_diedai;i++)
    {
        myfile<<fp[4+i].allcorners.at(0)<<" "<<fp[4+i].allcorners.at(29)<<" "<<fp[4+i].allcorners.at(30)<<" "<<fp[4+i].allcorners.at(49)<<endl;


        target_pose= arm.getCurrentPose().pose;

        cv::Mat dy=cv::Mat(3,1,CV_64F);
        dy.at<double>(0)=fp[0].allcorners.at(0).x-fp[i+4].allcorners.at(0).x;
        dy.at<double>(1)=fp[0].allcorners.at(0).y-fp[i+4].allcorners.at(0).y;
        dy.at<double>(2)=fp[0].allcorners.at(30).x-fp[i+4].allcorners.at(30).x;

        cv::Mat dp=cv::Mat(3,1,CV_64F);
        dp = pos_con.J_evlt.reshape(0,3).inv()*dy;

        cout<<"J"<<endl<<pos_con.J_evlt.reshape(0,3)<<endl;


        target_pose.position.x=target_pose.position.x+r*dp.at<double>(0);
        target_pose.position.y=target_pose.position.y+r*dp.at<double>(1);
        target_pose.position.z=target_pose.position.z+r*dp.at<double>(2);

        rob.move_robots(arm,target_pose);

        if(rob.getData()==false) return 1;
        images.push_back(rob.image);
        frames.push_back(rob.frame);
        fp[5+i].init(images.at(5+i),frames.at(5+i));

        vector<double> dy_v;
        vector<double> dp_v;

        dy_v.push_back(fp[i+5].allcorners.at(0).x-fp[i+4].allcorners.at(0).x);
        dy_v.push_back(fp[i+5].allcorners.at(0).y-fp[i+4].allcorners.at(0).y);
        dy_v.push_back(fp[i+5].allcorners.at(30).x-fp[i+4].allcorners.at(30).x);

        dp_v.push_back(fp[i+5].frame.p.x()-fp[i+4].frame.p.x());
        dp_v.push_back(fp[i+5].frame.p.y()-fp[i+4].frame.p.y());
        dp_v.push_back(fp[i+5].frame.p.z()-fp[i+4].frame.p.z());



        pos_con.KalmanFilter(dy_v,dp_v);

    }

    return 0;
}


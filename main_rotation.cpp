#include "thirdprocessor.h"
#include "position_controller.h"

void rotation_control(geometry_msgs::Pose &target_pose,firstprocessor r_fp,firstprocessor r_fp_goal,const cv::Mat J_rotation)
{
    cv::Mat R_real,R_sim,H_sim;
    secondprocessor test;
    test.init(r_fp,r_fp_goal);
    test.getdr();
    test.ComputeH21();
    R_real = test.dr;
    H_sim=test.H;
    R_sim = J_rotation.inv()*H_sim*J_rotation;

    cv::Mat u,aw,vt;

    SVD::compute(R_sim,aw,u,vt);

    R_sim = u*vt;


    KDL::Rotation r(R_sim.at<double>(0),R_sim.at<double>(1),R_sim.at<double>(2),
                    R_sim.at<double>(3),R_sim.at<double>(4),R_sim.at<double>(5),
                    R_sim.at<double>(6),R_sim.at<double>(7),R_sim.at<double>(8));

    KDL::Rotation goal_r;
    goal_r = r_fp.frame.M*r;

    double x,y,z,w;
    goal_r.GetQuaternion(x,y,z,w);
    target_pose.orientation.x=x;
    target_pose.orientation.y=y;
    target_pose.orientation.z=z;
    target_pose.orientation.w=w;

}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "lunwen");
    ros::AsyncSpinner spinner(5);
    spinner.start();

//**************************
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

//**************************

//**************************
    ofstream myfile("/home/leixiaoyu/桌面/data/r_data/data_result.txt");
    string str;
    int number_picture=0;

    if (!myfile.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }
    myfile.clear();
//**************************

//**************************
    geometry_msgs::Pose target_pose;
    firstprocessor r_fp_goal;
    robot rob;
    firstprocessor r_fp;
    firstprocessor r_fp_pre;
    thirdprocessor tp;
//**************************

//**************************
//    std::string path= "/home/leixiaoyu/桌面/pic/goal.jpg";
//    r_fp_goal.init(imread(path.c_str() , -1),KDL::Frame::Identity());

    if(rob.getData()==false) return 1;
    r_fp_goal.init(rob.image,rob.frame);

//    char a;
//    cin>>a;
//**************************

    int sum =1;
    vector<cv::Mat> images;
    vector<KDL::Frame> frams;
    firstprocessor fp;


    target_pose= arm.getCurrentPose().pose;
    target_pose.position.x = target_pose.position.x+0.2;
    target_pose.position.y = target_pose.position.y+0.2;
    target_pose.position.z = target_pose.position.z+0.2;
    rob.move_robots(&arm,target_pose);



    cv::Mat K(3, 3, CV_64FC1);
    K = (cv::Mat_<double>(3, 3) <<  1885.972653384125, 0, 1024.5,
            0, 1885.972653384125, 1024.5,
            0, 0, 1);
    cv::Mat R_sim,R_real;


    if(rob.getData()==false) return 1;
    r_fp.init(rob.image,rob.frame);
    secondprocessor sp;
    sp.init(r_fp,r_fp_goal);
    sp.getdr();
    sp.ComputeH21();

    R_sim = K.inv()*sp.H*K;
    R_real = sp.dr;

    cv::Mat u,aw,vt;

    SVD::compute(R_sim,aw,u,vt);

    cout<<"R_sim"<<endl<<R_sim<<endl;

    R_sim = u*vt;

    cout<<"R_sim"<<endl<<R_sim<<endl;
    cout<<"R_real"<<endl<<R_real<<endl;

////**************************
//    KDL::Rotation r[4] ;
//    r[0] = Rotation::RotX(0.1);
//    r[1] = Rotation::RotY(0.1);
//    r[2] = Rotation::RotZ(0.1);
//    int sum =3;
//    for(int i=0;i<sum;i++)
//    {
//        {
//            {
//                if(rob.getData()==false) return 1;
//
//                if(i==0)
//                {
//                    r_fp.init(rob.image,rob.frame);
//                }
//
//                else
//                {
//                    r_fp_pre.init(r_fp.image,r_fp.frame);
//                    r_fp.init(rob.image,rob.frame);
//
//                    secondprocessor sp;
//                    sp.init(r_fp_pre,r_fp);
//                    sp.getdr();
//                    sp.ComputeH21();
//
//                    tp.push_back(sp);
//                }
//
//                if(i==3) break;
//
//                target_pose= arm.getCurrentPose().pose;
//                Rotation rx=Rotation::Quaternion(target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);
//                KDL::Rotation goal_r;
//                goal_r = rx*r[i];
//
//                double x,y,z,w;
//
//                goal_r.GetQuaternion(x,y,z,w);
//                target_pose.orientation.x=x;
//                target_pose.orientation.y=y;
//                target_pose.orientation.z=z;
//                target_pose.orientation.w=w;
//
//                rob.move_robots(arm,target_pose);
//
//            }
//        }
//    }
////**************************
//
////**************************
//    myfile<<r_fp_goal.allcorners.at(0)<<" "<<r_fp_goal.allcorners.at(29)<<" "<<r_fp_goal.allcorners.at(30)<<" "<<r_fp_goal.allcorners.at(49)<<endl;
////**************************
//
////**************************
//    int diedai = 1000;
//    for(int i=0;i<diedai;i++)
//    {
//        //**************************
//        tp.getJ_rotation();
//
//        target_pose= arm.getCurrentPose().pose;
//
//        rotation_control(target_pose,r_fp,r_fp_goal,tp.J_rotation);
//
//        rob.move_robots(arm,target_pose);
//
//        if(rob.getData()==false) return 1;
//        r_fp_pre.init(r_fp.image,r_fp.frame);
//        r_fp.init(rob.image,rob.frame);
//
//        secondprocessor sp;
//        sp.init(r_fp,r_fp_pre);
//        sp.getdr();
//        sp.ComputeH21();
//        tp.push_back(sp);
//
//        myfile<<r_fp.allcorners.at(0)<<" "<<r_fp.allcorners.at(29)<<" "<<r_fp.allcorners.at(30)<<" "<<r_fp.allcorners.at(49)<<endl;
//        if(i == 2 )
//        {
//            myfile<<endl;
//            cout<<"-------------------------------------------------------------"<<endl;
//        }
//
//        if(i > 2 )
//        {
//            vector<secondprocessor>::iterator kk = tp.sps.begin();
//            tp.sps.erase(kk);
//        }
//        //**************************
//
//    }
////**************************

    return 0;
}

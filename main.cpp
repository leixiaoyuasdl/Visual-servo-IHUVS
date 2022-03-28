//
// Created by lxy on 2022/2/21.
//
#include "thirdprocessor.h"
int readdata(vector<cv::Mat> &images,vector<KDL::Frame> &frames)
{
    ifstream myfile("/home/leixiaoyu/桌面/data/r_data/data.txt");
    string str;
    int number_picture=0;

    if (!myfile.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }

    for(int i=0;i<10;i++)
    {
        double x,y,z,w;
        myfile >> x >> y >> z >>w;
        cout<<x<<" "<<y<<" "<<z<<" "<<w<<endl;

        KDL::Frame f;
        f.M = Rotation::Quaternion(x,y,z,w);
        frames.push_back(f);

        std::string image_name = "image"+std::to_string(number_picture)+".jpg";
        std::string path = "/home/leixiaoyu/桌面/data/pic/" + image_name;

        images.push_back(imread(path.c_str() , -1));
        number_picture++;
    }

    myfile.close();

    return number_picture;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "lunwen");
    ros::AsyncSpinner spinner(5);
    spinner.start();
//    sensor_msgs::CameraInfoConstPtr cam_info;
//    cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/ir/camera_info",ros::Duration(30));
//
//    cout<<"aa  "<<endl;
//
//    cv::Mat K(3, 3, CV_64FC1);
//    K.setTo(0);
//    K.at<double>(0,0) = cam_info->P[0];   K.at<double>(0,1) = cam_info->P[1];   K.at<double>(0,2) = cam_info->P[2];
//    K.at<double>(1,0) = cam_info->P[4];   K.at<double>(1,1) = cam_info->P[5];   K.at<double>(1,2) = cam_info->P[6];
//    K.at<double>(2,0) = cam_info->P[8];   K.at<double>(2,1) = cam_info->P[9];   K.at<double>(2,2) = cam_info->P[10];
//
//    cout<<"K   "<<K<<endl;

    cv::Mat K(3, 3, CV_64FC1);
    K = (cv::Mat_<double>(3, 3) <<  1885.972653384125, 0, 1024.5,
    0, 1885.972653384125, 1024.5,
    0, 0, 1);

    robot rob;
    vector<cv::Mat> images;
    vector<KDL::Frame> frames;
   // readdata(images,frames);

    int sum =4;
    for(int i=0;i<sum;i++)
    {
        char y;
        while (1)
        {
            cout<<"is ok?"<<endl;
            cin>>y;
            if(y=='y')
            {
                if(rob.getData()==false) return 0;
                images.push_back(rob.image);
                frames.push_back(rob.frame);
                break;
            }
        }
    }


    firstprocessor fp[100];
    for(int i=0;i<4;i++)
    {
        fp[i].init(images.at(i),frames.at(i));
    }


    //test
    cv::Mat R_real,R_sim,H_sim;
    secondprocessor test;
    int start=0;
    int end =3;

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
    geometry_msgs::Pose target_pose;

    int diedai = 100;
    thirdprocessor tp;
    for(int j=end-2;j<end;j++)
    {
        secondprocessor sp;
        sp.init(fp[j],fp[j+1]);
        sp.getdr();
        sp.ComputeH21();
        tp.push_back(sp);
    }

    for(int i=0;i<diedai;i++)
    {

        tp.getJ_rotation();
        cout<<"J -"<<tp.J_rotation<<endl;


        test.init(fp[end],fp[start]);
        test.getdr();
        test.ComputeH21();
        R_real = test.dr;
        H_sim=test.H;
        R_sim = tp.J_rotation.inv()*H_sim*tp.J_rotation;

        cv::Mat u,aw,vt;

        SVD::compute(R_sim,aw,u,vt);

        R_sim = u*vt;

//        cout<<"R_real"<<endl;
//        cout<<R_real<<endl;
//
//        cout<<"R_sim_1"<<endl;
//        cout<<R_sim<<endl;

        target_pose= arm.getCurrentPose().pose;

        KDL::Rotation r(R_sim.at<double>(0),R_sim.at<double>(1),R_sim.at<double>(2),
                        R_sim.at<double>(3),R_sim.at<double>(4),R_sim.at<double>(5),
                        R_sim.at<double>(6),R_sim.at<double>(7),R_sim.at<double>(8));

        KDL::Rotation goal_r;
        goal_r = fp[end].frame.M*r;

        double x,y,z,w;
        goal_r.GetQuaternion(x,y,z,w);
        target_pose.orientation.x=x;
        target_pose.orientation.y=y;
        target_pose.orientation.z=z;
        target_pose.orientation.w=w;



        rob.move_robots(arm,target_pose);
        end++;
        if(rob.getData()==false) return 1;
        images.push_back(rob.image);
        frames.push_back(rob.frame);
        fp[end].init(images.at(end),frames.at(end));

        secondprocessor sp;
        sp.init(fp[end],fp[end-1]);
        sp.getdr();
        sp.ComputeH21();
        tp.push_back(sp);

    }
    return 0;
}


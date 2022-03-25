//
// Created by lxy on 2022/2/21.
//
#include "thirdprocessor.h"
int readdata(robot a[])
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
        a[number_picture].frame.M = Rotation::Quaternion(x,y,z,w);

        std::string image_name = "image"+std::to_string(number_picture)+".jpg";
        std::string path = "/home/leixiaoyu/桌面/data/pic/" + image_name;
        a[number_picture].image = imread(path.c_str() , -1);
        number_picture++;
    }

    myfile.close();

    return number_picture;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "lunwen");

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

    robot rob[20];
  //  readdata(rob);

    int sum =3;
    for(int i=0;i<sum;i++)
    {
        char y;
        while (1)
        {
            cout<<"is ok?"<<endl;
            cin>>y;
            if(y=='y')
            {
                if(rob[i].getData()==false) return 0;
                break;
            }
        }
    }


    firstprocessor fp[10];
    for(int i=0;i<3;i++)
    {
        fp[i].init(rob[i]);
        cout<<"frame  "<<fp[i].da.frame<<endl;
       cout<<"---x y "<<fp[i].mvKeys<<endl;
    }


    thirdprocessor tp;
    for(int i=0;i<sum-1;i++)
    {
        secondprocessor sp;
        sp.init(fp[i],fp[i+1]);
        sp.getdr();
        sp.ComputeH21();
        cout<<"H   "<<sp.H<<endl;
//        cout<<"KRK^-1 "<<K*sp.dr*K.inv()<<endl;

        cout<<"dr-----"<<sp.dr<<endl;

       // sp.H = K*sp.dr*K.inv();
        tp.push_back(sp);
      //  cout<<sp.H.inv()<<endl;
    }

    tp.getJ_rotation();

    cout<<"J -"<<tp.J_rotation<<endl;

    //test
    cv::Mat R_real,R_sim,H_sim;
//    secondprocessor test;
//    int goal=0;
//    int start =1;
//    test.init(fp[start],fp[goal]);
//    test.getdr();
//    test.ComputeH21();
//    R_real = test.dr;
//    H_sim=test.H;
//    R_sim = tp.J_rotation.inv()*H_sim*tp.J_rotation;
//
//    cout<<"R_real"<<endl;
//    cout<<R_real<<endl;
//
//    cout<<"R_sim"<<endl;
//    cout<<R_sim<<endl;

    cv::Mat H_real;
    R_real = (cv::Mat_<double>(3, 3) <<  1.0000000,  0.0000000,  0.0000000,
            0.0000000, -0.8390715,  0.5440211,
            0.0000000, -0.5440211, -0.8390715);
    H_real=K*R_real*K.inv();
    R_sim = tp.J_rotation.inv()*H_real*tp.J_rotation;

    cout<<"R_real"<<endl;
    cout<<R_real<<endl;

    cout<<"R_sim"<<endl;
    cout<<R_sim<<endl;


    return 0;
}


//
// Created by lxy on 2022/2/21.
//
#include "thirdprocessor.h"
int readdata(data *a)
{
    ifstream myfile("/home/lxy/experiment/data/data.txt");
    string str;
    int number_picture=0;
    int number_point=0;

    if (!myfile.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }

    while(myfile >> str)
    {
        if(str == "sum_pitures")
        {
            int sum;
            myfile >> sum;
            number_picture = sum;
            number_point=0;
        }
        else if(str == "frame_p")
        {
            double x,y,z;
            myfile >> x >> y >> z;
            a[number_picture].v.data[0] = x;
            a[number_picture].v.data[1] = y;
            a[number_picture].v.data[2] = z;
        }
        else if(str == "frame_r")
        {
            double x,y,z,w;
            myfile >> x >> y >> z >>w;
            a[number_picture].r = Rotation::Quaternion(x,y,z,w);
        }
        else
        {
            a[number_picture].sum.push_back(stoi(str));

            double x,y;
            myfile >> x >> y ;

            a[number_picture].x.push_back(x);
            a[number_picture].y.push_back(y);

            number_point++;
        }
    }

    myfile.close();

    return number_picture;
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "lunwen");

    sensor_msgs::CameraInfoConstPtr cam_info;
    cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/ir/camera_info",ros::Duration(30));

    cout<<"aa  "<<endl;

    cv::Mat K(3, 3, CV_64FC1);
    K.setTo(0);
    K.at<double>(0,0) = cam_info->P[0];   K.at<double>(0,1) = cam_info->P[1];   K.at<double>(0,2) = cam_info->P[2];
    K.at<double>(1,0) = cam_info->P[4];   K.at<double>(1,1) = cam_info->P[5];   K.at<double>(1,2) = cam_info->P[6];
    K.at<double>(2,0) = cam_info->P[8];   K.at<double>(2,1) = cam_info->P[9];   K.at<double>(2,2) = cam_info->P[10];

    cout<<"K   "<<K<<endl;
    std::string refFrame="base_link";
    std::string childFrame="camera_ir_optical_frame";



    data da[10];
//    readdata(da);
    refFrame="base_link";
    childFrame="tool0";

    robot rob(refFrame,childFrame);

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
                if(rob.getData(da[i])==false) return 0;
                break;
            }
        }
        for(int j=0;j<da[i].y.size();j++)
        {
            cout<<"xy "<<i<<" "<<da[i].x.at(j)<<" "<<da[i].y.at(j)<<endl;
        }
    }



    Size PatSize1,PatSize2;
    PatSize1.width = 7;
    PatSize1.height = 6;
    PatSize2.width = 6;
    PatSize2.height = 5;

    firstprocessor fp[10];
    for(int i=0;i<sum;i++)
    {
        fp[i].init(da[i]);
        fp[i].GetFourPoints(PatSize1,0);
        //fp[i].GetFourPoints(PatSize2,PatSize1.width*PatSize1.height);

//        fp[i].mvKeys.clear();
//        for(int j=0;j<da[i].x.size();j++)
//        {
//            cv::Point2f Key;
//            Key.x = da[i].x.at(j);
//            Key.y = da[i].y.at(j);
//            fp[i].mvKeys.push_back(Key);
//        }
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
        cout<<"KRK^-1 "<<K*sp.dr*K.inv()<<endl;
//        cout<<"dr-----"<<sp.dr<<endl;
        tp.push_back(sp);
      //  cout<<sp.H.inv()<<endl;
    }

    tp.getJ_rotation();

    cout<<"J -"<<tp.J_rotation.t()<<endl;
//    //验证
//    for(int i=0;i<2;i++)
//    {
//        cv::Mat R_real,R_sim;
//        R_real = tp.sps.at(i).dr;
//        R_sim = tp.J_rotation.t().inv()*tp.sps.at(i).H*tp.J_rotation.t();
//        cout<<i<<endl;
//        cout<<"R_real"<<endl;
//
//        cout<<R_real<<endl;
//        cout<<"R_sim"<<endl;
//        cout<<R_sim<<endl;
//
//    }
    cv::Mat R_real,R_sim,H_real;
    R_real = (cv::Mat_<double>(3, 3) <<  1.0000000,  0.0000000,  0.0000000,
            0.0000000, -0.8390715,  0.5440211,
            0.0000000, -0.5440211, -0.8390715);
    H_real=K*R_real*K.inv();
        R_sim = tp.J_rotation.t().inv()*H_real*tp.J_rotation.t();
        cout<<"R_real"<<endl;

        cout<<R_real<<endl;
        cout<<"R_sim"<<endl;
        cout<<R_sim<<endl;


    return 0;
}


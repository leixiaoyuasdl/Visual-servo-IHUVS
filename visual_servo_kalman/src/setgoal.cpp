//
// Created by lxy on 2022/2/21.
//
#include "robot.h"
#include <ros/package.h>
int main(int argc,char **argv)
{
    ros::init(argc, argv, "lunwen");

    std::string FileName = ros::package::getPath("visual_servo_kalman");


//**************************
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);
//*************************

    robot rob;

//**************************
    Mat img_goal;
    vector<cv::Point2f> allcorners_goal;
    std::string path_pic= FileName + "/goal/goal.jpg";

    if(rob.getData()==false) return 1;

    imwrite(path_pic.c_str(),rob.image);

    ofstream myfile(FileName + "/goal/frame.txt");
    if (!myfile.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }
    myfile.clear();

    double x,y,z,w;
    rob.frame.M.GetQuaternion(x,y,z,w);
    myfile<<rob.frame.p.x()<<" "<<rob.frame.p.y()<<" "<<rob.frame.p.z()<<endl;
    myfile<<x<<" "<<y<<" "<<z<<" "<<w<<endl;



    return 0;
}


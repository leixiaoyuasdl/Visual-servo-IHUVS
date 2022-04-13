//
// Created by lxy on 2022/2/21.
//
#include "controller.h"
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
    ofstream myfile("/home/leixiaoyu/桌面/data/r_data/data.txt");
    if (!myfile.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }
    myfile.clear();
//**************************

//**************************
    controller con;
    con.arm.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));
//**************************

//**************************
    Mat img_goal;
    vector<cv::Point2f> allcorners_goal;
    std::string path= "/home/leixiaoyu/桌面/pic/goal.jpg";
    img_goal = imread(path.c_str() , -1);
    allcorners_goal = con.im_p.getallcorners(img_goal);
    img_goal.copyTo(con.img_goal);
    con.allcorners_goal = allcorners_goal;

//    if(con.rob.getData()==false) return 1;
//
//    allcorners_goal = con.im_p.getallcorners(con.rob.image);
//    con.rob.image.copyTo(con.img_goal);
//    con.allcorners_goal = allcorners_goal;


    myfile<<con.allcorners_goal.at(0)<<" "<<con.allcorners_goal.at(10)<<endl<<endl;
    con.rotation_init();
    con.rotation_control();

    con.position_init();
    int diedai=100;
    for(int i=0;i<diedai;i++)
    {
        con.position_control();
        myfile<<con.im_p.getallcorners(con.rob.image).at(0)<<" "<<con.im_p.getallcorners(con.rob.image).at(10)<<endl;

        con.rotation_control();
        myfile<<con.im_p.getallcorners(con.rob.image).at(0)<<" "<<con.im_p.getallcorners(con.rob.image).at(10)<<endl;
    }

    return 0;
}


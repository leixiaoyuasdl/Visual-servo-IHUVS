//
// Created by lxy on 2022/2/21.
//
#include "thirdprocessor.h"
#include "position_controller.h"
#include "controller/controller.h"
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
    firstprocessor fp_goal;
    controller con;
    con.arm.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));
//**************************

//**************************
    std::string path= "/home/leixiaoyu/桌面/pic/goal.jpg";
    fp_goal.init(imread(path.c_str() , -1),KDL::Frame::Identity());
    con.fp_goal = fp_goal;

//    if(con.rob.getData()==false) return 1;
//    fp_goal.init(con.rob.image,con.rob.frame);
//    con.fp_goal = fp_goal;
//
//    char a;
//    cin>>a;

//    myfile<<fp_goal.allcorners.at(0)<<" "<<fp_goal.allcorners.at(29)<<" "<<fp_goal.allcorners.at(30)<<" "<<fp_goal.allcorners.at(49)<<endl;
    myfile<<fp_goal.allcorners.at(0)<<" "<<fp_goal.allcorners.at(10)<<endl<<endl;
    con.rotation_init();
    //con.rotation_control(1);
    con.rotation_control(0);

    con.position_init();
    int diedai=100;
    for(int i=0;i<diedai;i++)
    {
        con.position_control();
        con.rotation_control(0);
  //      myfile<<con.fp.allcorners.at(0)<<" "<<con.fp.allcorners.at(29)<<" "<<con.fp.allcorners.at(30)<<" "<<con.fp.allcorners.at(49)<<endl;

        myfile<<con.fp.allcorners.at(0)<<" "<<con.fp.allcorners.at(10)<<endl;
    }


    return 0;
}


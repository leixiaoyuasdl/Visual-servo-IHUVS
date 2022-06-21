#include "controller_kalman_6dof.h"
#include <ros/package.h>
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

    std::string FileName = ros::package::getPath("visual_servo_kalman");

//**************************
    ofstream myfile(FileName  + "/data/data.txt");
    if (!myfile.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }
    myfile.clear();
//**************************

//**************************
    ofstream myfile_frame(FileName  +  "/data/frame.txt");
    if (!myfile_frame.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }
    myfile_frame.clear();
//**************************


//**************************
    controller_kalman_6dof con;
    con.arm.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));
//**************************




//**************************
    Mat img_goal;
    vector<cv::Point2f> allcorners_goal;
    std::string path= FileName  +  "/goal/goal.jpg";

    std::string path_start= FileName  + "/data/start.jpg";

    if(con.rob.getData()==false) return 1;
    imwrite(path_start.c_str(),con.rob.image);
//    char a;
//    cin>>a;

    img_goal = imread(path.c_str() , -1);
    allcorners_goal = con.im_p.getallcorners(img_goal);
    img_goal.copyTo(con.img_goal);
    con.allcorners_goal = allcorners_goal;



//    allcorners_goal = con.im_p.getallcorners(con.rob.image);
//    con.rob.image.copyTo(con.img_goal);
//    con.allcorners_goal = allcorners_goal;

//
//    myfile<<con.allcorners_goal.at(0)<<" "<<con.allcorners_goal.at(10)<<endl<<endl;

    myfile<<con.allcorners_goal.at(0).x<<" "<<con.allcorners_goal.at(0).y<<" ";
    myfile<<con.allcorners_goal.at(29).x<<" "<<con.allcorners_goal.at(29).y<<" ";
    myfile<<con.allcorners_goal.at(30).x<<" "<<con.allcorners_goal.at(30).y<<" ";
    myfile<<con.allcorners_goal.at(49).x<<" "<<con.allcorners_goal.at(49).y<<" "<<endl;

    double x,y,z,w;

    myfile<<con.im_p.getallcorners(con.rob.image).at(0).x<<" "<<con.im_p.getallcorners(con.rob.image).at(0).y<<" ";
    myfile<<con.im_p.getallcorners(con.rob.image).at(29).x<<" "<<con.im_p.getallcorners(con.rob.image).at(29).y<<" ";
    myfile<<con.im_p.getallcorners(con.rob.image).at(30).x<<" "<<con.im_p.getallcorners(con.rob.image).at(30).y<<" ";
    myfile<<con.im_p.getallcorners(con.rob.image).at(49).x<<" "<<con.im_p.getallcorners(con.rob.image).at(49).y<<" "<<endl;


    con.rob.frame.M.GetQuaternion(x,y,z,w);
    myfile_frame<<con.rob.frame.p.x()<<" "<<con.rob.frame.p.y()<<" "<<con.rob.frame.p.z()<<endl;
    myfile_frame<<x<<" "<<y<<" "<<z<<" "<<w<<endl;

    con.position_init();
//    con.position_control();
    myfile<<con.im_p.getallcorners(con.rob.image).at(0).x<<" "<<con.im_p.getallcorners(con.rob.image).at(0).y<<" ";
    myfile<<con.im_p.getallcorners(con.rob.image).at(29).x<<" "<<con.im_p.getallcorners(con.rob.image).at(29).y<<" ";
    myfile<<con.im_p.getallcorners(con.rob.image).at(30).x<<" "<<con.im_p.getallcorners(con.rob.image).at(30).y<<" ";
    myfile<<con.im_p.getallcorners(con.rob.image).at(49).x<<" "<<con.im_p.getallcorners(con.rob.image).at(49).y<<" "<<endl;


    con.rob.frame.M.GetQuaternion(x,y,z,w);
    myfile_frame<<con.rob.frame.p.x()<<" "<<con.rob.frame.p.y()<<" "<<con.rob.frame.p.z()<<endl;
    myfile_frame<<x<<" "<<y<<" "<<z<<" "<<w<<endl;

    int diedai=100;



    for(int i=0;i<diedai;i++)
    {
//        if(i>3)
//            con.k=0.8;

        con.position_control();

        myfile<<con.im_p.getallcorners(con.rob.image).at(0).x<<" "<<con.im_p.getallcorners(con.rob.image).at(0).y<<" ";
        myfile<<con.im_p.getallcorners(con.rob.image).at(29).x<<" "<<con.im_p.getallcorners(con.rob.image).at(29).y<<" ";
        myfile<<con.im_p.getallcorners(con.rob.image).at(30).x<<" "<<con.im_p.getallcorners(con.rob.image).at(30).y<<" ";
        myfile<<con.im_p.getallcorners(con.rob.image).at(49).x<<" "<<con.im_p.getallcorners(con.rob.image).at(49).y<<" "<<endl;

        con.rob.frame.M.GetQuaternion(x,y,z,w);
        myfile_frame<<con.rob.frame.p.x()<<" "<<con.rob.frame.p.y()<<" "<<con.rob.frame.p.z()<<endl;
        myfile_frame<<x<<" "<<y<<" "<<z<<" "<<w<<endl;
    }

    return 0;
}


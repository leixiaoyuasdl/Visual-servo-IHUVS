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

    std::string FileName = ros::package::getPath("visual_servo");

//**************************
    ofstream myfile(FileName + "/data/data.txt");
    if (!myfile.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }
    myfile.clear();
//**************************

//**************************
    ofstream myfile_frame(FileName + "/data/frame.txt");
    if (!myfile_frame.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }
    myfile_frame.clear();
//**************************

//**************************
    ifstream myfile_goal(FileName + "/goal/frame.txt");
    if (!myfile_goal.is_open())
    {
        cout << "未成功打开文件" << endl;
        return 0;
    }
//**************************

//**************************
    double goal_x,goal_y,goal_z;
    double goal_rx,goal_ry,goal_rz,goal_rw;
    myfile_goal>>goal_x>>goal_y>>goal_z;
    myfile_goal>>goal_rx>>goal_ry>>goal_rz>>goal_rw;
    geometry_msgs::Pose target_pose;

    double t=0.4;

    target_pose.position.x = goal_x;
    target_pose.position.y = goal_y;
    target_pose.position.z = goal_z ;

    target_pose.position.x = target_pose.position.x - 0.2*t;
    target_pose.position.y = target_pose.position.y - 0.3*t;
    target_pose.position.z = target_pose.position.z + 0.4*t;

    KDL::Rotation r_goal = Rotation::Quaternion(goal_rx,goal_ry,goal_rz,goal_rw);

    KDL::Rotation r = Rotation::Identity();

    r =  Rotation::RPY(20.0*t/180*M_PI,-20.0*t/180*M_PI,60.0*t/180*M_PI);
    //r =  Rotation::RPY(-10.0/180*M_PI,10.0/180*M_PI,30.0/180*M_PI);

    r_goal= r_goal*r;

    r_goal.GetQuaternion(target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = 0.0;
    int maxtries = 100;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        attempts++;

        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }

    if(fraction == 1)
    {
        ROS_INFO("Path computed successfully. Moving the arm.");

        // 生成机械臂的运动规划数据
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        // 执行运动
        arm.execute(plan);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

//**************************

//**************************
    controller_kalman_6dof con;
    con.arm.reset(new moveit::planning_interface::MoveGroupInterface("manipulator"));

//**************************


//**************************
    Mat img_goal;
    vector<cv::Point2f> allcorners_goal;
    std::string path= FileName + "/goal/goal.jpg";

    std::string path_start= FileName + "/data/start.jpg";

    if(con.rob.getData()==false) return 1;
    imwrite(path_start.c_str(),con.rob.image);

    img_goal = imread(path.c_str() , -1);
    allcorners_goal = con.im_p.getallcorners(img_goal);
    img_goal.copyTo(con.img_goal);
    con.allcorners_goal = allcorners_goal;

    int sign[5];
    sign[0]=0;sign[1]=29;sign[2]=30;sign[3]=49;
    double x,y,z,w;


    for(int i=0;i<4;i++)
        myfile<<con.allcorners_goal.at(sign[i]).x<<" "<<con.allcorners_goal.at(sign[i]).y<<" ";
    myfile<<endl;


    for(int i=0;i<4;i++)
        myfile<<con.im_p.getallcorners(con.rob.image).at(sign[i]).x<<" "<<con.im_p.getallcorners(con.rob.image).at(sign[i]).y<<" ";
    myfile<<endl;

    con.rob.frame.M.GetQuaternion(x,y,z,w);
    myfile_frame<<con.rob.frame.p.x()<<" "<<con.rob.frame.p.y()<<" "<<con.rob.frame.p.z()<<endl;
    myfile_frame<<x<<" "<<y<<" "<<z<<" "<<w<<endl;

    con.position_init();

    int diedai=20;
    for(int i=0;i<diedai;i++)
    {
        con.position_control();

        for(int j=0;j<4;j++)
            myfile<<con.im_p.getallcorners(con.rob.image).at(sign[j]).x<<" "<<con.im_p.getallcorners(con.rob.image).at(sign[j]).y<<" ";
        myfile<<endl;

        con.rob.frame.M.GetQuaternion(x,y,z,w);
        myfile_frame<<con.rob.frame.p.x()<<" "<<con.rob.frame.p.y()<<" "<<con.rob.frame.p.z()<<endl;
        myfile_frame<<x<<" "<<y<<" "<<z<<" "<<w<<endl;
    }


    return 0;
}



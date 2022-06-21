//
// Created by lxy on 2022/2/21.
//
#include "controller.h"
#include <ros/package.h>
int main(int argc,char **argv)
{

    std::string FileName = ros::package::getPath("visual_servo");
//**************************
    ofstream myfile(FileName + "/data/error.txt");
    if (!myfile.is_open())
    {
        cout << "未成功打开文件 error" << endl;
        return 0;
    }
    myfile.clear();
//**************************

//**************************
    ofstream myfile_f_error(FileName + "/data/f_error.txt");
    if (!myfile_f_error.is_open())
    {
        cout << "未成功打开文件 f error" << endl;
        return 0;
    }
    myfile_f_error.clear();
//**************************

//**************************
    ifstream myfile_frame(FileName + "/data/frame.txt");
    if (!myfile_frame.is_open())
    {
        cout << "未成功打开文件 frame" << endl;
        return 0;
    }
//**************************

//**************************
    ifstream myfile_goal(FileName + "/goal/frame.txt");
    if (!myfile_goal.is_open())
    {
        cout << "未成功打开文件 goal frame" << endl;
        return 0;
    }
//**************************

//**************************
    ifstream myfile_data(FileName + "/data/data.txt");
    if (!myfile_data.is_open())
    {
        cout << "未成功打开文件 data" << endl;
        return 0;
    }
//**************************

    double x_goal,y_goal,z_goal;
    double rx_goal,ry_goal,rz_goal,rw_goal;
    myfile_goal>>x_goal>>y_goal>>z_goal;
    myfile_goal>>rx_goal>>ry_goal>>rz_goal>>rw_goal;

    int sum=40;
    double x,y,z;
    double rx,ry,rz,rw;

    double dp;
    double dr;
    double dpx,dpy;

    double px[5],py[5];
    double px_goal[5],py_goal[5];

    for(int i=0;i<4;i++)
        myfile_data>>px_goal[i]>>py_goal[i];

    for(int i=0;i<sum;i++)
    {
        for(int j=0;j<4;j++)
        {
            myfile_data>>px[j]>>py[j];
            dpx = px[j]-px_goal[j];
            dpy = py[j]-py_goal[j];
            myfile_f_error<<dpx<<" "<<dpy<<" ";
        }
        myfile_f_error<<endl;
    }

    KDL::Rotation r_goal,r;
    r_goal = Rotation::Quaternion(rx_goal,ry_goal,rz_goal,rw_goal);

    for(int i=0;i<sum;i++)
    {
        myfile_frame>>x>>y>>z;
        myfile_frame>>rx>>ry>>rz>>rw;
        r = Rotation::Quaternion(rx,ry,rz,rw);

        dp = sqrt((x-x_goal)*(x-x_goal)+(y-y_goal)*(y-y_goal)+(z-z_goal)*(z-z_goal));
        //dr = sqrt((rx-rx_goal)*(rx-rx_goal)+(ry-ry_goal)*(ry-ry_goal)+(rz-rz_goal)*(rz-rz_goal)+(rw-rw_goal)*(rw-rw_goal));
        Vector axis;
        dr = (r.Inverse()*r_goal).GetRotAngle(axis);
        cout<<axis<<endl;
        cout<<axis.x()*axis.x()+axis.y()*axis.y()+axis.z()*axis.z()<<endl;

        myfile<<dp<<" "<<dr<<endl;
    }
    return 0;
}


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

    data da[10];
    readdata(da);

//    const std::string refFrame="base_link";
//    const std::string childFrame="tool0";
//
//    robot rob(refFrame,childFrame);
//
//    for(int i=0;i<3;i++)
//    {
//        char y;
//        while (1)
//        {
//            cout<<"is ok?"<<endl;
//            cin>>y;
//            if(y=='y')
//            {
//                if(rob.getData(da[i])==false) return 0;
//                break;
//            }
//        }
//    }

    int sum =3;
    firstprocessor fp[10];
    for(int i=0;i<sum;i++)
    {
        fp[i].init(da[i]);
        fp[i].GetFourPoints();
    }


    thirdprocessor tp;
    for(int i=0;i<sum-1;i++)
    {
        secondprocessor sp;
        sp.init(fp[i],fp[i+1]);
        sp.getdr();
        sp.ComputeH21();
        tp.push_back(sp);
        cout<<sp.H<<endl;
    }

    tp.getJ_rotation();

    //验证
    for(int i=0;i<2;i++)
    {
        cv::Mat R_real,R_sim;
        R_real = tp.sps.at(i).dr;
        R_sim = tp.J_rotation.inv()*tp.sps.at(i).H*tp.J_rotation;
        cout<<i<<endl;
        cout<<"R_real"<<endl;

        cout<<R_real<<endl;
        cout<<"R_sim"<<endl;
        cout<<R_sim<<endl;

    }

    return 0;
}


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
    ros::init(argc, argv, "");

    const std::string refFrame="base_link";
    const std::string childFrame="tool0";

    robot rob;
    rob.getTransform(refFrame,childFrame);
    rob.getFeatures();

    data da[10];
    for(int i=0;i<3;i++)
    {
        da[i] = rob.getData();
    }

    firstprocessor fp[10];
    for(int i=0;i<3;i++)
    {
        fp[i].init(da[i]);
        fp[i].GetFourPoints();
    }

    int start,end;
    start =0;
    end =1;

    thirdprocessor tp;
    for(int i=0;i<2;i++)
    {
        secondprocessor sp;
        sp.init(fp[i],fp[i+1]);
        sp.ComputeH21();
        tp.push_back(sp);
    }

    tp.getJ_rotation();

    return 0;
}


//
// Created by lxy on 2022/2/21.
//
#include "firstprocessor.h"
#include "secondprocessor.h"
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


int main()
{
    int number_picture = 0;
    data da[10];

    number_picture = readdata(da);

    firstprocessor fp[10];
    for(int i=0;i<2;i++)
    {
        fp[i].init(da[i]);
        fp[i].GetFourPoints();
    }

    int start,end;
    start =0;
    end =1;

    secondprocessor sp;
    sp.init(fp[start],fp[end]);
    sp.ComputeH21();

    cout<<" H1 "<<endl;
    cout<<sp.H<<endl;


    double det = cv::determinant(sp.H);

    cout<<det<<endl;

    double k=pow(det,1.0/3.0);

    cout<<" H2 "<<endl;
    cout<<sp.H*(1.0/k)<<endl;

    return 0;
}


#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <opencv2/opencv.hpp>
double REJECT_DEGREE_TH = 4.0;

using namespace std;
using namespace KDL;
using namespace cv;

struct data
{
    vector<int> sum;
    vector<double> x;
    vector<double> y;
    KDL::Vector v;
    KDL::Rotation r;

};

struct point
{
    double x;
    double y;
};

struct picture
{
    std::vector<point> p;
};


std::vector<std::vector<double>> FilterLines(const std::vector<std::vector<double> > &Lines) {
    std::vector<std::vector<double> > FinalLines;

    for (int i = 0; i < Lines.size(); i++)
    {
        std::vector<double> Line = Lines[i];
        double x1 = Line[0], y1 = Line[1];
        double x2 = Line[2], y2 = Line[3];

        double m, c;

        // Calculating equation of the line : y = mx + c
        if (x1 != x2)
            m = (double)(y2 - y1) / (double)(x2 - x1);
        else
        {
            m = 100000000.0;
            cout<<"m= 10000000000"<<endl;
        }

        c = y2 - m * x2;

        // theta will contain values between - 90 -> + 90.

        /*# Rejecting lines of slope near to 0 degree or 90 degree and storing others
        if REJECT_DEGREE_TH <= abs(theta) <= (90 - REJECT_DEGREE_TH):
            l = math.sqrt( (y2 - y1)**2 + (x2 - x1)**2 )    # length of the line
            FinalLines.append([x1, y1, x2, y2, m, c, l])*/
        // Rejecting lines of slope near to 0 degree or 90 degree and storing others
        //if (REJECT_DEGREE_TH <= abs(theta) && abs(theta) <= (90.0 - REJECT_DEGREE_TH))
//		{
			double l = pow((pow((y2 - y1), 2) + pow((x2 - x1), 2)), 0.5);	// length of the line
			std::vector<double> FinalLine{ (double)x1, (double)y1, (double)x2, (double)y2, m, c, l };
			FinalLines.push_back(FinalLine);
//		}
    }

    // Removing extra lines
    // (we might get many lines, so we are going to take only longest 15 lines
    // for further computation because more than this number of lines will only
    // contribute towards slowing down of our algo.)

    return FinalLines;
}


int* GetVanishingPoint(std::vector<std::vector<double> > Lines)
{
	// We will apply RANSAC inspired algorithm for this.We will take combination
	// of 2 lines one by one, find their intersection point, and calculate the
	// total error(loss) of that point.Error of the point means root of sum of
	// squares of distance of that point from each line.
	int* VanishingPoint = new int[2];
	VanishingPoint[0] = -1; VanishingPoint[1] = -1;

	double MinError = 1000000000.0;

	for (int i = 0; i < Lines.size(); i++)
	{
		for (int j = i + 1; j < Lines.size(); j++)
		{
			double m1 = Lines[i][4], c1 = Lines[i][5];
			double m2 = Lines[j][4], c2 = Lines[j][5];

			if (m1 != m2)
			{
				double x0 = (c1 - c2) / (m2 - m1);
				double y0 = m1 * x0 + c1;

				double err = 0;
                for (int i = 0; i < Lines.size(); i++)
                {
                    std::vector<double> Line = Lines[i];
					double m = Line[4], c = Line[5];
					double m_ = (-1 / m);
					double c_ = y0 - m_ * x0;

					double x_ = (c - c_) / (m_ - m);
					double y_ = m_ * x_ + c_;

					double l = pow((pow((y_ - y0), 2) + pow((x_ - x0), 2)), 0.5);

					err += pow(l, 2);
				}

				err = pow(err, 0.5);

				if (MinError > err)
				{
					MinError = err;
					VanishingPoint[0] = (int)x0;
					VanishingPoint[1] = (int)y0;
				}
			}
            else
            {
//                int point_max=100000000;
//                if(m1 == 0)
//                {
//                    VanishingPoint[0] = point_max;
//                    VanishingPoint[1] = 0;
//                }
//                else
//                if(m1 >= point_max)
//                {
//                    VanishingPoint[0] = 0;
//                    VanishingPoint[1] = point_max;
//                }
//                else
//                {
//                    VanishingPoint[0] = pow(point_max/(1+m1*m1), 0.5);
//                    VanishingPoint[1] = m1*VanishingPoint[0];
//                }
                cout<<"m1 = m2--------------------------"<<endl;
            }
		}
	}

	return VanishingPoint;
}



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

bool findlines(int a,std::vector<std::vector<double> > &Lines,const data &da,const int* const map)
{
    for(int i=0;i<8;i++)
    {
        bool start=false;
        std::vector<double> line;
        for(int j=i*9;j<da.sum.size();j=j+a)
        {
            if(map[j]>-1)
            {
                if(start == false)
                {
                    line.push_back(da.x[map[j]]);
                    line.push_back(da.y[map[j]]);
                    start =true;
                }
                else
                {
                    line.push_back(da.x[map[j]]);
                    line.push_back(da.y[map[j]]);
                    Lines.push_back(line);
                    break;
                }
            }
        }
    }
    if(Lines.size()>=3) return true;
    else return false;
}
std::vector<double> getline(int a0,int a1,const data &da)
{
    std::vector<double> line;
    line.push_back(da.x[a0]);
    line.push_back(da.y[a0]);
    line.push_back(da.x[a1]);
    line.push_back(da.y[a1]);

    return line;
}


void GetFourPoints_test(picture &pic,const data &da,const int* const map)
{

    int* a[4];

    for(int i=0;i<4;i++)
    {
        std::vector<std::vector<double> > Lines;
        if(findlines(10+i,Lines,da,map)==true)
        {
            a[i] = GetVanishingPoint(FilterLines(Lines));
        }
        else
        {
            cout<<"error------------------"<<endl;
        }
    }

    for(int i=0;i<4;i++)
    {
        point pp;
        pp.x=a[i][0];
        pp.y=a[i][1];
        pic.p.push_back(pp);
    }

}

void GetFourPoints(picture &pic,const data &da,const int* const map)
{
    int* a[4];
    std::vector<std::vector<double> > Lines[10];
    int sum =4;
    int sumj=3;
    int sum_point=16;

    for(int i=0;i<sum;i++)
        for(int j=0;j<sumj;j++)
            Lines[0].push_back(getline(4*i,4*i+j+1,da));
    a[0] = GetVanishingPoint(FilterLines(Lines[0]));

    for(int i=0;i<sum;i++)
        for(int j=0;j<sumj;j++)
            Lines[1].push_back(getline(i,i+4*(j+1),da));
    a[1] = GetVanishingPoint(FilterLines(Lines[1]));


    for(int i=0;i<sum;i++)
        for(int j=0;j<sumj;j++)
            Lines[2].push_back(getline(sum_point+4*i,sum_point+4*i+j+1,da));
    a[2] = GetVanishingPoint(FilterLines(Lines[2]));


    for(int i=0;i<sum;i++)
        for(int j=0;j<sumj;j++)
            Lines[3].push_back(getline(sum_point+i,sum_point+i+4*(j+1),da));
    a[3] = GetVanishingPoint(FilterLines(Lines[3]));


    for(int i=0;i<4;i++)
    {
        point pp;
        pp.x=a[i][0];
        pp.y=a[i][1];
        pic.p.push_back(pp);
    }

//    for(int i=0;i<4;i++)
//    {
//        point pp;
//        pp.x=da.x[i];
//        pp.y=da.y[i];
//        pic.p.push_back(pp);
//    }

}

void Normalize(const vector<cv::KeyPoint> &vKeys,
               vector<cv::Point2f> &vNormalizedPoints,
               cv::Mat &T)
{
    double meanX = 0;
    double meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    double meanDevX = 0;
    double meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;

    double sX = 1.0/meanDevX;
    double sY = 1.0/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    T = cv::Mat::eye(3,3,CV_64F);
    T.at<double>(0,0) = sX;
    T.at<double>(1,1) = sY;
    T.at<double>(0,2) = -meanX*sX;
    T.at<double>(1,2) = -meanY*sY;
}

void Normalize_test(const vector<cv::KeyPoint> &vKeys,
               vector<cv::Point2f> &vNormalizedPoints)
{

    const int N = vKeys.size();
    vNormalizedPoints.resize(N);
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x ;
        vNormalizedPoints[i].y = vKeys[i].pt.y ;
    }


}

cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();
    cv::Mat A(2*N,9,CV_64F);

    for(int i=0; i<N; i++)
    {
        const double u1 = vP1[i].x;
        const double v1 = vP1[i].y;
        const double u2 = vP2[i].x;
        const double v2 = vP2[i].y;

        A.at<double>(2*i,0) = 0.0;
        A.at<double>(2*i,1) = 0.0;
        A.at<double>(2*i,2) = 0.0;
        A.at<double>(2*i,3) = -u1;
        A.at<double>(2*i,4) = -v1;
        A.at<double>(2*i,5) = -1;
        A.at<double>(2*i,6) = v2*u1;
        A.at<double>(2*i,7) = v2*v1;
        A.at<double>(2*i,8) = v2;

        A.at<double>(2*i+1,0) = u1;
        A.at<double>(2*i+1,1) = v1;
        A.at<double>(2*i+1,2) = 1;
        A.at<double>(2*i+1,3) = 0.0;
        A.at<double>(2*i+1,4) = 0.0;
        A.at<double>(2*i+1,5) = 0.0;
        A.at<double>(2*i+1,6) = -u2*u1;
        A.at<double>(2*i+1,7) = -u2*v1;
        A.at<double>(2*i+1,8) = -u2;

    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);


    return vt.row(8).reshape(0, 3);
}

cv::Mat ComputeH21_test(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();
    cv::Mat A(2*N,8,CV_64F);
    cv::Mat b(2*N,1,CV_64F);

    for(int i=0; i<N; i++)
    {
        const double u1 = vP1[i].x;
        const double v1 = vP1[i].y;
        const double u2 = vP2[i].x;
        const double v2 = vP2[i].y;

        A.at<double>(2*i,0) = u1;
        A.at<double>(2*i,1) = v1;
        A.at<double>(2*i,2) = 1;
        A.at<double>(2*i,3) = 0;
        A.at<double>(2*i,4) = 0;
        A.at<double>(2*i,5) = 0;
        A.at<double>(2*i,6) = -u1*u2;
        A.at<double>(2*i,7) = -v1*u2;

        A.at<double>(2*i+1,0) = 0;
        A.at<double>(2*i+1,1) = 0;
        A.at<double>(2*i+1,2) = 0;
        A.at<double>(2*i+1,3) = u1;
        A.at<double>(2*i+1,4) = v1;
        A.at<double>(2*i+1,5) = 1;
        A.at<double>(2*i+1,6) = -u1*v2;
        A.at<double>(2*i+1,7) = -v1*v2;

        b.at<double>(2*i,0) = u2;
        b.at<double>(2*i+1,0) = v2;
    }

    cv::Mat x(9,1,CV_64F);
    cv::solve(A,b,x(Rect(0,0,1,8)),CV_SVD);

    x.at<double>(8,0) = 1;
    return x.reshape(0, 3);
}

//void FindHomography(vector<bool> &vbMatchesInliers, double &score, cv::Mat &H21)
//{
//    //归一化后的参考帧和当前帧中的特征点坐标
//    vector<cv::Point2f> vPn1, vPn2;// 2d-2d点对
//    //各自的归一化矩阵
//    //其实这里的矩阵归一化操作主要是为了在单目初始化过程中，固定场景的尺度，原理可以参考SLAM十四讲P152
//    cv::Mat T1, T2;// 标准化矩阵
//    Normalize(mvKeys1,vPn1, T1);// 标准化点坐标  去均值点坐标 * 绝对矩倒数
//    Normalize(mvKeys2,vPn2, T2);
//    //这里求的逆在后面的代码中要用到，辅助进行原始尺度的恢复
//    cv::Mat T2inv = T2.inv();// 标准化矩阵 逆矩阵
//    cv::Mat H21i, H12i;// 原点对的单应矩阵 //  H21i 原始点
//    ///此处略去了随机取8对对应约束的过程，vPn1i,vPn2i即迭代过程中选取的对应点对。
//    cv::Mat Hn = ComputeH21(vPn1,vPn2);//  计算标准化后的点对的单应矩阵
//    // 去归一化，恢复原始的均值和尺度
//    H21i = T2inv*Hn*T1;  // 原始点    p1 ---> p2 的单应
//    H12i = H21i.inv();   // 原始点    p2 ---> p1 的单应
//}

int main() {
//	std::vector<cv::Mat> Images;			// Input Images will be stored in this list.
//	std::vector<std::string> ImageNames;	// Names of input images will be stored in this list.
//	ReadImage("InputImages", Images, ImageNames);
//
//	for (int i = 0; i < Images.size(); i++)
//	{
//		cv::Mat Image = Images[i].clone();
//
//		// Getting the lines form the image
//		std::vector<std::vector<double>> Lines;
//		Lines = GetLines(Image);
//
//		// Get vanishing point
//		int* VanishingPoint = GetVanishingPoint(Lines);
//
//		// Checking if vanishing point found
//		if (VanishingPoint[0] == -1 && VanishingPoint[1] == -1)
//		{
//			std::cout << "Vanishing Point not found. Possible reason is that not enough lines are found in the image for determination of vanishing point." << std::endl;
//			continue;
//		}
//
//		// Drawing linesand vanishing point
//		for (int i = 0; i < Lines.size(); i++)
//		{
//			std::vector<double> Line = Lines[i];
//			cv::line(Image, cv::Point((int)Line[0], (int)Line[1]), cv::Point((int)Line[2], (int)Line[3]), cv::Scalar(0, 255, 0), 2);
//		}
//		cv::circle(Image, cv::Point(VanishingPoint[0], VanishingPoint[1]), 10, cv::Scalar(0, 0, 255), -1);
//
//		// Showing the final image
//		cv::imshow("OutputImage", Image);
//		cv::waitKey(0);
//	}

    int number_picture = 0;
    data da[10];


    number_picture = readdata(da);

    cout<<number_picture<<endl;
    int map[10][100];
    for (int i = 0; i < 10; i++)
        for (int j = 0; j < 100; j++)
            map[i][j] = -1;

    for (int i = 0; i <= number_picture; i++) {
        for (int j = 0; j < da[i].sum.size(); j++) {
            map[i][da[i].sum[j]] = j;
            cout<<da[i].sum[j]<<" "<<da[i].x[j]<<" "<<da[i].y[j]<<endl;
        }
         cout<<endl;
    }


    vector<cv::KeyPoint> mvKeys[10];
    for (int i=0;i<=number_picture;i++)
    {
        picture pic;
        GetFourPoints(pic,da[i],map[i]);
        for(int j=0;j<pic.p.size();j++)
        {
            cv::KeyPoint p;
            p.pt.x = pic.p[j].x;
            p.pt.y = pic.p[j].y;
            mvKeys[i].push_back(p);
            cout<<"X:"<<pic.p[j].x<<" Y:"<<pic.p[j].y<<endl;
            //cout<<"p      "<<p.pt<<endl;
        }
        cout<<endl;
    }


    int start,end;
    start =0;
    end =1;


    //归一化后的参考帧和当前帧中的特征点坐标
    vector<cv::Point2f> vPn1, vPn2;// 2d-2d点对
    //各自的归一化矩阵
    //其实这里的矩阵归一化操作主要是为了在单目初始化过程中，固定场景的尺度，原理可以参考SLAM十四讲P152
    cv::Mat T1, T2;// 标准化矩阵


//    Normalize(mvKeys[start],vPn1,T1);// 标准化点坐标  去均值点坐标 * 绝对矩倒数
//    Normalize(mvKeys[end],vPn2,T2);
    Normalize_test(mvKeys[start],vPn1);// 标准化点坐标  去均值点坐标 * 绝对矩倒数
    Normalize_test(mvKeys[end],vPn2);

    //这里求的逆在后面的代码中要用到，辅助进行原始尺度的恢复
//    cv::Mat T2inv = T2.inv();// 标准化矩阵 逆矩阵
//    cv::Mat H21i, H12i;// 原点对的单应矩阵 //  H21i 原始点
    //此处略去了随机取8对对应约束的过程，vPn1i,vPn2i即迭代过程中选取的对应点对。
    cv::Mat Hn = ComputeH21_test(vPn1,vPn2);//  计算标准化后的点对的单应矩阵

    cout<<" H "<<endl;
    cout<<Hn<<endl;
    double det = cv::determinant(Hn);

    cout<<det<<endl;

    double k=pow(det,1.0/3.0);

    cout<<" H2 "<<endl;
    cout<<Hn*(1.0/k)<<endl;

    det = cv::determinant(Hn*(1.0/k));
    cout<<det<<endl;

    // 去归一化，恢复原始的均值和尺度
//    H21i = T2inv*Hn*T1;  // 原始点    p1 ---> p2 的单应
//    H12i = H21i.inv();   // 原始点    p2 ---> p1 的单应
//
//    cout<<" H21 "<<endl;
//    cout<<H21i<<endl;
//
//    cout<<" H12 "<<endl;
//    cout<<H12i<<endl;

    KDL::Rotation K(604.93402, 0, 314.42896,
                    0, 604.80316, 234.56125,
                    0, 0, 1);

//    KDL::Rotation H(H21i.data[0], H21i.data[1], H21i.data[2],
//                    H21i.data[3], H21i.data[4], H21i.data[5],
//                    H21i.data[6], H21i.data[7], H21i.data[8]);

    KDL::Rotation R;
    R=da[start].r.Inverse()*da[end].r;

    cv::Mat R_opencv(3,3,CV_64F);
    for(int i=0;i<9;i++)
    {
        R_opencv.at<double>(i/3,i%3) = R.data[i];
    }

    cv::Mat K_opencv(3,3,CV_64F);
    for(int i=0;i<9;i++)
    {
        K_opencv.at<double>(i/3,i%3) = K.data[i];
    }
  //  cout<<R_opencv<<endl;
    cv::Mat HH;
    HH=K_opencv*R_opencv.inv()*K_opencv.inv();

//    cout<<"R_opencv"<<endl;
//    cout<<R_opencv<<endl;

    cout<<"HH"<<endl;
    cout<<HH<<endl;

//    cout<<"Hn*HH.inv()"<<endl;
//    cout<<Hn*HH.inv()<<endl;



    return 0;
}
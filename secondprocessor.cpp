//
// Created by lxy on 2022/2/21.
//

#include "secondprocessor.h"
void secondprocessor::ComputeH21()
{
    vector<cv::Point2f> vP1,vP2;
    vP1 = fp1.mvKeys;
    vP2 = fp2.mvKeys;

//    const int N = vP1.size();
//    cv::Mat A(2*N,8,CV_64F);
//    cv::Mat b(2*N,1,CV_64F);
//
//    for(int i=0; i<N; i++)
//    {
//        const double u1 = vP1[i].x;
//        const double v1 = vP1[i].y;
//        const double u2 = vP2[i].x;
//        const double v2 = vP2[i].y;
//
//        A.at<double>(2*i,0) = u1;
//        A.at<double>(2*i,1) = v1;
//        A.at<double>(2*i,2) = 1;
//        A.at<double>(2*i,3) = 0;
//        A.at<double>(2*i,4) = 0;
//        A.at<double>(2*i,5) = 0;
//        A.at<double>(2*i,6) = -u1*u2;
//        A.at<double>(2*i,7) = -v1*u2;
//
//        A.at<double>(2*i+1,0) = 0;
//        A.at<double>(2*i+1,1) = 0;
//        A.at<double>(2*i+1,2) = 0;
//        A.at<double>(2*i+1,3) = u1;
//        A.at<double>(2*i+1,4) = v1;
//        A.at<double>(2*i+1,5) = 1;
//        A.at<double>(2*i+1,6) = -u1*v2;
//        A.at<double>(2*i+1,7) = -v1*v2;
//
//        b.at<double>(2*i,0) = u2;
//        b.at<double>(2*i+1,0) = v2;
//    }
//
//    cv::Mat x(9,1,CV_64F);
//    cv::solve(A,b,x(Rect(0,0,1,8)),CV_SVD);
//
//    x.at<double>(8,0) = 1;
//    x.reshape(0, 3).copyTo(H);

    H = findHomography(vP1, vP2,0);


    double det = cv::determinant(H);

    double k=pow(det,1.0/3.0);

    H=H*(1.0/k);
}

void secondprocessor::getdr()
{
    KDL::Rotation rr;
    rr = fp1.da.r.Inverse()*fp2.da.r;
    cv::Mat a(3,3,CV_64F,rr.data);
    a.copyTo(dr);
}

void secondprocessor::init(firstprocessor a, firstprocessor b)
{
    fp1 = a;
    fp2 = b;
}
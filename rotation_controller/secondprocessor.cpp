//
// Created by lxy on 2022/2/21.
//

#include "secondprocessor.h"
void secondprocessor::ComputeH21()
{
    vector<cv::Point2f> vP1,vP2;
//    vP1 = fp1.mvKeys;
//    vP2 = fp2.mvKeys;

    vP1 = fp1.allcorners;
    vP2 = fp2.allcorners;

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


    cv::Mat T1, T2;// 标准化矩阵
    vector<cv::Point2f> vPn1, vPn2;

    Normalize(vP1,vPn1, T1);// 标准化点坐标  去均值点坐标 * 绝对矩倒数
    Normalize(vP2,vPn2, T2);


    findHomography(vPn2,vPn1,0).copyTo(H);

    H = T1.inv()*H*T2;  // 原始点    p1 ---> p2 的单应
    //H = H.inv();   // 原始点    p2 ---> p1 的单应
    double det = cv::determinant(H);

    double k=pow(det,1.0/3.0);

    H=H*(1.0/k);
}

void secondprocessor::ComputeH21_chu()
{
    vector<cv::Point2f> vP1,vP2;
    vP1 = fp1.mvKeys;
    vP2 = fp2.mvKeys;

    cv::Mat T1, T2;// 标准化矩阵
    vector<cv::Point2f> vPn1, vPn2;

    Normalize(vP1,vPn1, T1);// 标准化点坐标  去均值点坐标 * 绝对矩倒数
    Normalize(vP2,vPn2, T2);


    findHomography(vPn2,vPn1,0).copyTo(H);

    H = T1.inv()*H*T2;  // 原始点    p1 ---> p2 的单应
    //H = H.inv();   // 原始点    p2 ---> p1 的单应
    double det = cv::determinant(H);

    double k=pow(det,1.0/3.0);

    H=H*(1.0/k);
}

void secondprocessor::getdr()
{
    KDL::Rotation rr;
    rr = fp1.frame.M.Inverse()*fp2.frame.M;
    cv::Mat a(3,3,CV_64F,rr.data);
    a.copyTo(dr);
}

void secondprocessor::init(firstprocessor &a, firstprocessor &b)
{
    fp1 = a;
    fp2 = b;

}

void secondprocessor::Normalize(const vector<cv::Point2f> &vKeys, vector<cv::Point2f> &vNormalizedPoints,cv::Mat &T)
{
    double meanX = 0;
    double meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].x;
        meanY += vKeys[i].y;
    }

    meanX = meanX/N;
    meanY = meanY/N;

    double meanDevX = 0;
    double meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].x - meanX;
        vNormalizedPoints[i].y = vKeys[i].y - meanY;

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

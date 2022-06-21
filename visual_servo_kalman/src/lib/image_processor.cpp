//
// Created by leixiaoyu on 2022/4/13.
//

#include "image_processor.h"
void image_processor::getchessboardcorners(Mat src, Size PatSize)
{
    vector<Point2f> corners;

    Mat gray;

    src.copyTo(gray);

    //cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
//    threshold(src,gray,90,255,THRESH_BINARY);

//    imshow("aa",gray);
//    waitKey(0);
    bool found=findChessboardCorners(gray, PatSize, corners);
    if (!found) {
        cerr << "find corners failured!" << endl;
        exit(1);
    }

    cv::TermCriteria criteria = cv::TermCriteria(
            cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
            40000,
            0.0001);

    //亚像素检测
    cv::cornerSubPix(gray, corners, cv::Size(10, 10), cv::Size(-1, -1), criteria);

    allcorners.insert(allcorners.end(),corners.begin(),corners.end());

//    cout << "find corners" << endl;
//    drawChessboardCorners(gray, PatSize, corners, found);


}

vector<cv::Point2f> image_processor::getallcorners(Mat img)
{
    allcorners.clear();

    Size PatSize1,PatSize2;

    PatSize1.width = 6;
    PatSize1.height = 5;
    PatSize2.width = 5;
    PatSize2.height = 4;

    getchessboardcorners(img,PatSize1);
    getchessboardcorners(img,PatSize2);

    return allcorners;
}

void image_processor::Normalize(const vector<cv::Point2f> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
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

cv::Mat image_processor::ComputeH21(cv::Mat img1, cv::Mat img2)
{
    vector<cv::Point2f> vP1,vP2;
    cv::Mat H;

    vP1 = getallcorners(img1);
    vP2 = getallcorners(img2);

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

    return H;
}
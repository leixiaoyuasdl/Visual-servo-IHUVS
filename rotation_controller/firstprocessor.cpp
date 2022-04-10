//
// Created by LeiXiaoYu on 2022/2/21.
//

#include "firstprocessor.h"

void firstprocessor::init(cv::Mat im,KDL::Frame f)
{
    vector<cv::Point2f>().swap(mvKeys);
    vector<cv::Point2f>().swap(allcorners);
    im.copyTo(image);
    frame = f;
    GetFourPoints();
}
void firstprocessor::GetFourPoints()
{
    double* a[10];

    Size PatSize1,PatSize2;

    PatSize1.width = 6;
    PatSize1.height = 5;
    PatSize2.width = 5;
    PatSize2.height = 4;

    getchessboardcorners(image,PatSize1);
    getchessboardcorners(image,PatSize2);

//    std::vector<std::vector<double> > Lines[10];
//
//    int sum=6;
//
//    for(int i=0;i<sum;i++)
//        Lines[0].push_back(getline(0+i,24+i));
//    a[0] = GetVanishingPoint(FilterLines(Lines[0]));
//
//    for(int i=0;i<sum-1;i++)
//        Lines[1].push_back(getline(0+i,24+(i+1)));
//    a[1] = GetVanishingPoint(FilterLines(Lines[1]));
//
////    for(int i=0;i<sum-2;i++)
////        Lines[4].push_back(getline(0+i,24+(i+2)));
////    a[4] = GetVanishingPoint(FilterLines(Lines[4]));
////
////    for(int i=0;i<sum-1;i++)
////        Lines[5].push_back(getline(24+i,0+(i+1)));
////    a[5] = GetVanishingPoint(FilterLines(Lines[5]));
////
////    for(int i=0;i<sum-2;i++)
////        Lines[6].push_back(getline(24+i,0+(i+2)));
////    a[6] = GetVanishingPoint(FilterLines(Lines[6]));
//
//
//    for(int i=0;i<sum-2;i++)
//        Lines[2].push_back(getline(30+0+i*5,30+4+i*5));
//    a[2] = GetVanishingPoint(FilterLines(Lines[2]));
//
//    for(int i=0;i<sum-3;i++)
//        Lines[3].push_back(getline(30+0+i*5,30+4+(i+1)*5));
//    a[3] = GetVanishingPoint(FilterLines(Lines[3]));
//
////    for(int i=0;i<sum-4;i++)
////        Lines[7].push_back(getline(30+0+i*5,30+4+(i+2)*5));
////    a[7] = GetVanishingPoint(FilterLines(Lines[7]));
////
////    for(int i=0;i<sum-3;i++)
////        Lines[8].push_back(getline(30+4+(i)*5,30+0+(i+1)*5));
////    a[8] = GetVanishingPoint(FilterLines(Lines[8]));
////
////    for(int i=0;i<sum-4;i++)
////        Lines[9].push_back(getline(30+4+(i)*5,30+0+(i+2)*5));
////    a[9] = GetVanishingPoint(FilterLines(Lines[9]));
//
////    Lines[0].push_back(getline(0,4));
////        Lines[0].push_back(getline(15,19));
////    a[0] = GetVanishingPoint(FilterLines(Lines[0]));
////
////    Lines[1].push_back(getline(0,15));
////        Lines[1].push_back(getline(4,19));
////    a[1] = GetVanishingPoint(FilterLines(Lines[1]));
////
////    Lines[2].push_back(getline(20+0,20+8));
////        Lines[2].push_back(getline(20+3,20+11));
////    a[2] = GetVanishingPoint(FilterLines(Lines[2]));
////
////    Lines[3].push_back(getline(20+0,20+3));
////        Lines[3].push_back(getline(20+8,20+11));
////    a[3] = GetVanishingPoint(FilterLines(Lines[3]));
//
//    for(int i=0;i<4;i++)
//    {
//        cv::Point2f p;
//        p.x = a[i][0];
//        p.y = a[i][1];
//        mvKeys.push_back(p);
//    }
}

std::vector<double> firstprocessor::getline(int a0,int a1)
{
    std::vector<double> line;
    line.push_back(allcorners.at(a0).x);
    line.push_back(allcorners.at(a0).y);
    line.push_back(allcorners.at(a1).x);
    line.push_back(allcorners.at(a1).y);

    return line;
}



double *firstprocessor::GetVanishingPoint(std::vector<std::vector<double> > Lines)
{
    double* VanishingPoint = new double[2];
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
                    VanishingPoint[0] = x0;
                    VanishingPoint[1] = y0;
                }
            }
            else
            {

                cout<<"m1 = m2--------------------------"<<endl;
            }
        }
    }

    return VanishingPoint;
}

std::vector<std::vector<double>> firstprocessor::FilterLines(const std::vector<std::vector<double>> &Lines)
{
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

void firstprocessor::getchessboardcorners(Mat src, Size PatSize)
{
//    Mat src1;
////    imwrite("/home/fzt/桌面/pics/1.jpg",src);
//    //src = imread("/home/leixiaoyu/桌面/pics/1.jpg", -1);//载入测试图像
//    if (src.channels() == 1)
//    {
//        src1 = src.clone();
//    }
//    else
//    {
//        if (src.channels() == 3)
//        {
//            cv::cvtColor(src, src1, CV_BGR2GRAY);
//        }
//        else
//        {
//            if (src.channels() == 4)
//            {
//                cv::cvtColor(src, src1, CV_BGRA2GRAY);
//            }
//        }
//    }
//
//    vector<Point> corners_p;//存储找到的角点
//
//    double t = (double)getTickCount();
//    std::vector<cv::Mat> chessboards;
//    CornerDetAC corner_detector(src1);
//    ChessboradStruct chessboardstruct;
//
//    Corners corners_s;
//    corner_detector.detectCorners(src1, corners_p, corners_s, 0.01);
//
//    t = ((double)getTickCount() - t) / getTickFrequency();
//    std::cout << "time cost :" << t << std::endl;
//
//    ImageChessesStruct ics;
//    chessboardstruct.chessboardsFromCorners(corners_s, chessboards, 0.6);
//    chessboardstruct.drawchessboard(src1, corners_s, chessboards, "cb", 0);
//
//    cout<<"size0 "<<chessboards.at(0).size<<endl;
//    cout<<"size1 "<<chessboards.at(1).size<<endl;
//    for(int i=0;i<chessboards.at(0).size[1];i++)
//        for(int j=0;j<chessboards.at(0).size[0];j++)
//        {
//            da.x.push_back(corners_s.p.at(chessboards.at(0).at<int>(j,i)).x);
//            da.y.push_back(corners_s.p.at(chessboards.at(0).at<int>(j,i)).y);
//        }
//
//    for(int i=0;i<chessboards.at(1).size[1];i++)
//        for(int j=0;j<chessboards.at(1).size[0];j++)
//        {
//            da.x.push_back(corners_s.p.at(chessboards.at(1).at<int>(j,i)).x);
//            da.y.push_back(corners_s.p.at(chessboards.at(1).at<int>(j,i)).y);
//        }
//
////            for(int i=0;i<10;i++)
////        {
////            cout<<corners_s.p.at(i).x<<" "<<corners_s.p.at(i).y<<endl;
////
////        }
//
//    Point center;
//    cv::Mat disp = src1.clone();
//    cv::RNG rng(0xFFFFFFFF);
//    cv::Scalar s(rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0), rng.uniform(0.0, 1.0));
//    s = s * 255;
//
//    int n = 8;
//    float scale = 0.3;
//    if (disp.rows < 2000 || disp.cols < 2000)
//    {
//        scale = 1;
//        n = 2;
//    }
//        for(int i=0;i<37;i++)
//    {
//            center.x = da.x.at(i);
//        center.y = da.y.at(i);
//        cv::circle(disp, center, n, Scalar(0, 255, 0),n);
//    }
//
//    cv::Mat SmallMat;
//    cv::resize(disp, SmallMat, cv::Size(), scale, scale);
//    cv::namedWindow("title");
//    cv::imshow("title", SmallMat);
//    cv::waitKey(0);



//    PatSize.width = 7;
//    PatSize.height = 6;
    vector<Point2f> corners;

    Mat gray;

    src.copyTo(gray);

    //cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    //threshold(src,gray_B,90,255,THRESH_BINARY);

    bool found=findChessboardCorners(gray, PatSize, corners);
    if (!found) {
        cout << "find corners failured!" << endl;
    }

    cv::TermCriteria criteria = cv::TermCriteria(
            cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
            40000,
            0.0001);

    //亚像素检测
    cv::cornerSubPix(gray, corners, cv::Size(10, 10), cv::Size(-1, -1), criteria);

    allcorners.insert(allcorners.end(),corners.begin(),corners.end());

    cout << "find corners" << endl;
    drawChessboardCorners(gray, PatSize, corners, found);
    cout << "draw corners" << endl;
//
//    for(int i=0;i<8;i++)
//    {
//        circle(gray, corners.at(i), 10, Scalar(0, 255, 0),-1);
//    }
//    cv::Mat SmallMat;
//    cv::resize(gray,SmallMat, cv::Size(), 0.3, 0.3);
//    imshow("chessboard corners",SmallMat);
//
//    waitKey(0);
}

//
// Created by lxy on 2022/2/28.
//

#include "robot.h"
robot::robot(const std::string reF, const std::string chF)
{
    refFrame=reF;
    childFrame=chF;
}
bool robot::getData(data &a)
{
    da.x.clear();
    da.y.clear();
    da.sum.clear();
//    if(getTransform()==false || getFeatures()==false)
//    {
//        return false;
//    }

    if(getTransform()==false) return false;
    if(getFeatures()==false) return false;


    a=da;
    return true;
}

bool robot::getTransform()
{
    tf::TransformListener _tfListener;
    tf::StampedTransform transform;
    std::string errMsg;

    if ( !_tfListener.waitForTransform(refFrame,
                                       childFrame,
                                       ros::Time(0),
                                       ros::Duration(200),
                                       ros::Duration(0.1),
                                       &errMsg)
            )
    {
        cout<<"is fasle 1 "<<errMsg<<endl;
        ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
        return false;
    }
    else
    {
        try
        {
            _tfListener.lookupTransform( refFrame, childFrame,
                                         ros::Time(0),                  //get latest available
                                         transform);
        }
        catch ( const tf::TransformException& e)
        {
            cout<<"is fasle 2"<<endl;
            ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
            return false;
        }

   }
    cout<<transform.getRotation()<<endl;
    Rotation rx=Rotation::Quaternion(transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    Vector vx(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    da.v = vx;
    da.r = rx;
    cout<<"------------"<<vx<<endl;
    return true;
}
void robot::getchessboardcorners(Mat src, Size PatSize)
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

    Mat gray,gray_B,src_copy;

    gray =src;

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


    for(int i=0;i<corners.size();i++)
    {
        da.x.push_back(corners.at(i).x);
        da.y.push_back(corners.at(i).y);
    }

    cout << "find corners" << endl;
    drawChessboardCorners(gray, PatSize, corners, found);
    cout << "draw corners" << endl;

    for(int i=0;i<8;i++)
    {
        circle(gray, corners.at(i), 10, Scalar(0, 255, 0),-1);
    }
    cv::Mat SmallMat;
    cv::resize(gray,SmallMat, cv::Size(), 0.3, 0.3);
    imshow("chessboard corners",SmallMat);

    waitKey(0);
}
bool robot::getFeatures()
{
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImageConstPtr image_sub;
    cv::Mat image,imagecopy;

    image_sub = ros::topic::waitForMessage<sensor_msgs::Image>("/camera/ir/image_raw",ros::Duration(3));
    if(image_sub != NULL)
    {
        cv_ptr = cv_bridge::toCvCopy(image_sub);
        image = cv_ptr -> image;
        //cv::imshow("image", image);
        ROS_INFO("image ok!");
        //cv::waitKey(0);
    }
    else
    {
        cout<<"no topic image_sub"<<endl;
        return false;
    }


    Size PatSize1,PatSize2;
    PatSize1.width = 6;
    PatSize1.height = 5;
    PatSize2.width = 5;
    PatSize2.height = 4;
    getchessboardcorners(image,PatSize1);
    getchessboardcorners(image,PatSize2);

    return true;


//    cv::Ptr<cv::aruco::DetectorParameters> params;
//
//    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
//    cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(10, 10, 0.015f, 0.01f, dictionary);
//
//    cout<<"--------------------"<<endl;
//
//    cv::Ptr<cv::aruco::Dictionary> dictionary1 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
//    cv::Ptr<cv::aruco::CharucoBoard> board1 = cv::aruco::CharucoBoard::create(5, 5, 0.02f, 0.015f, dictionary1);
//
//    params = cv::aruco::DetectorParameters::create();
//    image.copyTo(imagecopy);
//    std::vector<int> markerIds,markerIds1;
//    std::vector<std::vector<cv::Point2f> > markerCorners,markerCorners1;
//    cv::aruco::detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
//    //cv::aruco::detectMarkers(image, board1->dictionary, markerCorners1, markerIds1, params);
//
//    // if at least one marker detected
//    if (markerIds.size() > 0 )
//    {
//        //cv::aruco::drawDetectedMarkers(imagecopy, markerCorners, markerIds);
//        std::vector<cv::Point2f> charucoCorners;
//        std::vector<int> charucoIds;
//        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds);
//        // if at least one charuco corner detected
//        if (charucoIds.size() > 0)
//            cv::aruco::drawDetectedCornersCharuco(imagecopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
//        cv::imshow("image", imagecopy);
//
//        cv::waitKey(0);
////        std::vector<cv::Point2f> charucoCorners1;
////        std::vector<int> charucoIds1;
////        cv::aruco::interpolateCornersCharuco(markerCorners1, markerIds1, image, board1, charucoCorners1, charucoIds1);
////        // if at least one charuco corner detected
//////        if (charucoIds1.size() > 0)
//////            cv::aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners1, charucoIds1, cv::Scalar(255, 0, 0));
////
////        cout<<"-------------------  "<<charucoIds.size()<<endl;
////        cout<<"-------------------  "<<charucoIds1.size()<<endl;
//
//        for(int i=0;i<charucoIds.size();i++)
//        {
//            da.sum.push_back(charucoIds.at(i));
//            da.x.push_back(charucoCorners.at(i).x);
//            da.y.push_back(charucoCorners.at(i).y);
//        }
//
////        for(int i=0;i<charucoIds1.size();i++)
////        {
////            da.sum.push_back(charucoIds.at(i)+charucoIds1.size());
////            da.x.push_back(charucoCorners1.at(i).x);
////            da.y.push_back(charucoCorners1.at(i).y);
////        }
//    }
//    else return false;
//
//    return true;
}
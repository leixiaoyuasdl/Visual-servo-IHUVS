//
// Created by lxy on 2022/2/28.
//

#include "robot.h"
robot::robot()
{
    refFrame="base_link";
    childFrame="tool0";
}
bool robot::getData()
{

    if(getTransform()==false) return false;
    if(getPic()==false) return false;

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
    Rotation rx=Rotation::Quaternion(transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    Vector vx(transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
    frame.p = vx;
    frame.M = rx;

    return true;
}

bool robot::getPic()
{
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::ImageConstPtr image_sub;

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

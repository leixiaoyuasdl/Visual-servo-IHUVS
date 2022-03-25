//
// Created by lxy on 2022/2/21.
//

#include "thirdprocessor.h"

void thirdprocessor::push_back(secondprocessor a)
{
    sps.push_back(a);
}
cv::Mat thirdprocessor::kronecker(cv::Mat m1, cv::Mat m2)
{
    int mr = m1.rows;
    int mc = m1.cols;
    int nr = m2.rows;
    int nc = m2.cols;
    cv::Mat tt(mr*nr, mc * nc,CV_64F);
    if (mr == 0 || mc == 0 || nr == 0 || nc == 0) return tt;

    for (int i = 0; i < mr*nr; i++)
    {
        for (int j = 0; j < mc * nc; j++)
        {
            int a_i = i / mr;
            int a_j = j / mc;
            int b_i = i % mr;
            int b_j = j % mc;
            tt.at<double>(i,j)  = m1.at<double>(a_i,a_j) * m2.at<double>(b_i,b_j);
        }
    }
    return tt;
}

void thirdprocessor::getJ_rotation()
{
//    int sumofsps = sps.size();
//    cv::Mat x(sumofsps*9,9,CV_64F);
//    cv::Mat eye = Mat::eye(9,9,CV_64F);
//    for(int i=0;i<sumofsps;i++)
//    {
//        x(Rect(0,i*9,9,9))=eye - kronecker(sps.at(i).dr,sps.at(i).H);
//    }
//    cv::Mat u,w,vt;
//
//    //cv::SVDecomp(x,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
//    SVD::compute(x,w,u,vt);
//
////    cout<<"-----------------"<<endl;
////    cout<<x(Rect(0,9,9,9))*vt.row(8).reshape(0, 9)<<endl;
//
//    vt.row(8).reshape(0, 3).copyTo(J_rotation);
//    J_rotation =J_rotation.t();
    int sumofsps = sps.size();
    cv::Mat x(sumofsps*9,9,CV_64F);
    cv::Mat eye = Mat::eye(9,9,CV_64F);

    for(int i=0;i<sumofsps;i++)
    {
        cv::Mat a;
//        a=eye - kronecker(sps.at(i).dr,sps.at(i).H);
//        a.copyTo(x(Rect(0,i*9,9,9)));
        x(Rect(0,i*9,9,9))=eye - kronecker(sps.at(i).dr,sps.at(i).H);
    }

    cv::Mat u,w,vt;

    //cv::SVDecomp(x,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    SVD::compute(x,w,u,vt);

//    cout<<"-----------------"<<endl;
//    cout<<x(Rect(0,9,9,9))*vt.row(8).reshape(0, 9)<<endl;

    vt.row(8).reshape(0, 3).copyTo(J_rotation);
    J_rotation= J_rotation.t();
}
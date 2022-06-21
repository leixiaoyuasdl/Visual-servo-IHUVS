

#include "rotation_controller.h"
void rotation_controller::setJ0(vector<cv::Mat> Hs_r, vector<cv::Mat> drs_r)
{
    if( Hs_r.size() != drs_r.size() || Hs_r.size()<3 )
    {
        cerr << "size of Hs < 3" << endl;
        exit(3);
    }

    Hs = Hs_r;
    drs = drs_r;

    getJ();

}
void rotation_controller::add(cv::Mat dr, cv::Mat H)
{
    drs.push_back(dr);
    Hs.push_back(H);

    if(Hs.size() > size )
    {
        vector<cv::Mat>::iterator k1 = Hs.begin();
        Hs.erase(k1);

        vector<cv::Mat>::iterator k2 = drs.begin();
        drs.erase(k2);
    }
    getJ();
}

void rotation_controller::getJ()
{
    int sumofsps = Hs.size();
    cv::Mat x(sumofsps*9,9,CV_64F);
    cv::Mat eye = Mat::eye(9,9,CV_64F);
    for(int i=0;i<sumofsps;i++)
    {
        cv::Mat a;
        a=eye - kronecker(drs.at(i),Hs.at(i));
        a.copyTo(x(Rect(0,i*9,9,9)));
    }

    cv::Mat u,w,vt;

    SVD::compute(x,w,u,vt);

    vt.row(8).reshape(0, 3).copyTo(J);
    double k=J.at<double>(8);
    J= J.t()/k;
}

cv::Mat rotation_controller::kronecker(cv::Mat m1, cv::Mat m2)
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
//
// Created by LeiXiaoYu on 2022/2/21.
//

#include "firstprocessor.h"

void firstprocessor::init(data a)
{
    da = a;
}

void firstprocessor::setmap()
{
    for (int i = 0; i < 100; i++)
            map[i] = -1;

    for (int i = 0; i <= da.sum.size(); i++)
    {
            map[da.sum[i]] = -1;
    }
}

void firstprocessor::GetFourPoints(Size PatSize,int start)
{
    double* a[10];
    std::vector<std::vector<double> > Lines[10];


//    for(int i=0;i<PatSize.height;i++)
//        for(int j=2;j<PatSize.width;j++)
//            Lines[0].push_back(getline(start+PatSize.width*i,start+PatSize.width*i+j));
//    a[0] = GetVanishingPoint(FilterLines(Lines[0]));
//
//    for(int i=0;i<PatSize.width;i++)
//        for(int j=2;j<PatSize.height;j++)
//            Lines[1].push_back(getline(start+i,start+i+PatSize.width*j));
//    a[1] = GetVanishingPoint(FilterLines(Lines[1]));

//    for(int i=0;i<PatSize.height;i++)
//            Lines[0].push_back(getline(start+PatSize.width*i,start+PatSize.width*i+PatSize.width-1));
//    a[0] = GetVanishingPoint(FilterLines(Lines[0]));
//
//    for(int i=1;i<PatSize.height;i++)
//            Lines[1].push_back(getline(start+(PatSize.width)*(i)-1,start+(PatSize.width)*(i)));
//    a[1] = GetVanishingPoint(FilterLines(Lines[1]));
////    for(int i=0;i<PatSize.height-1;i++)
////        Lines[1].push_back(getline(start+(PatSize.width)*(i),start+(PatSize.width)*(i+2)-2));
////    a[1] = GetVanishingPoint(FilterLines(Lines[1]));
//
//    for(int i=0;i<PatSize.height-1;i++)
//        Lines[2].push_back(getline(start+(PatSize.width)*(i),start+(PatSize.width)*(i+2)-1));
//    a[2] = GetVanishingPoint(FilterLines(Lines[2]));

//    for(int i=1;i<PatSize.width;i++)
//    {
//        for(int j=0;j<PatSize.height-1;j++)
//        {
//            Lines[i-1].push_back(getline(start+(PatSize.width)*(j),start+(PatSize.width)*(j+2)-i));
//        }
//        a[i-1] = GetVanishingPoint(FilterLines(Lines[i-1]));
//
//    }
//
//    for(int i=1;i<PatSize.width;i++)
//    {
//        for(int j=1;j<PatSize.height;j++)
//        {
//            Lines[PatSize.width+i-1].push_back(getline(start+(PatSize.width)*(j),start+(PatSize.width)*(j)-i));
//        }
//        a[PatSize.width+i-1] = GetVanishingPoint(FilterLines(Lines[PatSize.width+i-1]));
//    }

//int sum=6;
//
//    for(int i=0;i<sum;i++)
//            Lines[0].push_back(getline(0+i*5,4+i*5));
//    a[0] = GetVanishingPoint(FilterLines(Lines[0]));
//
//    for(int i=0;i<sum-1;i++)
//            Lines[1].push_back(getline(0+i*5,4+(i+1)*5));
//    a[1] = GetVanishingPoint(FilterLines(Lines[1]));
//
//    for(int i=0;i<sum-2;i++)
//        Lines[2].push_back(getline(30+0+i,30+16+i));
//    a[2] = GetVanishingPoint(FilterLines(Lines[2]));
//
//    for(int i=0;i<sum-3;i++)
//        Lines[3].push_back(getline(30+0+i,30+16+(i+1)));
//    a[3] = GetVanishingPoint(FilterLines(Lines[3]));



    int sum=6;

    for(int i=0;i<sum;i++)
        Lines[0].push_back(getline(0+i,24+i));
    a[0] = GetVanishingPoint(FilterLines(Lines[0]));

    for(int i=0;i<sum-1;i++)
        Lines[1].push_back(getline(0+i,24+(i+1)));
    a[1] = GetVanishingPoint(FilterLines(Lines[1]));

    for(int i=0;i<sum-2;i++)
        Lines[2].push_back(getline(30+0+i*5,30+4+i*5));
    a[2] = GetVanishingPoint(FilterLines(Lines[2]));

    for(int i=0;i<sum-3;i++)
        Lines[3].push_back(getline(30+0+i*5,30+4+(i+1)*5));
    a[3] = GetVanishingPoint(FilterLines(Lines[3]));




    for(int i=0;i<4;i++)
    {
        cv::Point2f p;
        p.x = a[i][0];
        p.y = a[i][1];
        mvKeys.push_back(p);
    }
}

std::vector<double> firstprocessor::getline(int a0,int a1)
{
    std::vector<double> line;
    line.push_back(da.x[a0]);
    line.push_back(da.y[a0]);
    line.push_back(da.x[a1]);
    line.push_back(da.y[a1]);

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
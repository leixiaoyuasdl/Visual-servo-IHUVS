//
// Created by lxy on 2022/2/21.
//
#include <ros/package.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
using namespace std;
using namespace cv;

const int perBoardPixel = 100;  //定义每个格子的大小
const Size boardSize(5, 4);  //定义有几个格子
void createBoard(string filename) {

    //计算棋盘格的大小
    int height = perBoardPixel * boardSize.height;
    int width = perBoardPixel * boardSize.width;

    //定义一个Mat来存储棋盘格
    Mat board(height, width, CV_8UC1);

    //初始化棋盘格
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            board.at<uchar>(i, j) = 0;
        }
    }

    //奇数格子的为白色，偶数格子的为黑色
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            if ((i + j) % 2 == 1) {
                for (int r = i * perBoardPixel; r < (i + 1) * perBoardPixel; r++) {
                    for (int c = j * perBoardPixel; c < (j + 1) * perBoardPixel; c++) {
                        board.at<uchar>(r, c) = 255;
                    }
                }
            }
        }
    }

    imshow("chessboard", board);
    waitKey(0);


    //存储棋盘格图片
    imwrite(filename+"/chessboard/chessboard.png", board);


}

int main(int argc,char **argv)
{
    //生成棋盘格

    std::string FileName = ros::package::getPath("visual_servo");
    createBoard(FileName);

    return 0;
}


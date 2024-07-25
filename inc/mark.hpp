#pragma once


#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

#include "./armor.hpp"

cv::Mat thre(cv::Mat frame);
vector<armor::Armor> mark(cv::Mat frame_thre, cv::Mat frame);

cv::Mat imgSplitG(cv::Mat img);

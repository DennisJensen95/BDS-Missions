#include <stdio.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

using namespace std;

vector<vector<cv::Point2f>> wait_for_corners(int listen_seconds, const char *ip);
// Find ball class

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
//#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include "urun.h"
#include "ubridge.h"
#include "utime.h"
#include "ucamera.h"
#include "findball.h"

using namespace std;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////// FindBalls class ////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

int FindBalls::doFindBallProcessing(cv::Mat frame, int frameNumber, UTime imTime)
{
    vector<cv::Vec3d> rotationVectors, translationVectors;
    UTime t;
    t.now();

    // matrix to store gray picture
    cv::Mat gray;

    vector<cv::Vec3f> circles;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 21);

    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 1, 20, 40, 10, 50);

    if (circles.size() == 1){
        cv::Vec3i c = circles[0];
        int x = floor(c[0]);
        int y = floor(c[1]);
        int r = floor(c[2]);
        printf("Centre coordinates:\t");
        printf("x = %d, y = %d\n", x, y);
        printf("Radius:\t\t\tr = %d pixels\n", r);

        

    }

/*
    FindBall *v;
    v->lock.lock();
    v->imageTime = imTime;
    v->frameNumber = frameNumber;
    */
    return 0;
}

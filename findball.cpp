// Find ball class

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
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

    FindBall *v;
    v->lock.lock();
    v->imageTime = imTime;
    v->frameNumber = frameNumber;
    
    return EXIT_SUCCESS;
}
// Find a ball coordinates in robot coordinates

#ifndef UCAMERA_BALLFIND_H
#define UCAMERA_BALLFIND_H

#include <iostream>
#include <sys/time.h>
#include <thread>
// #include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
//#include <string>

#include "urun.h"
#include "ubridge.h"
#include "utime.h"
#include "ulibpose.h"
// #include "u2dline.h"
// this should be defined in the CMakeList.txt ? or ?
#ifdef raspicam_CV_LIBS
#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#endif

class FindBall
{
public:
    // Linux time when the image was captured
    UTime imageTime;

    int frameNumber = 0;
    /** Rotation    vector 
    * - x (left) ,y (down) ,z (forward), length is angle in radians.
    * - as found by cv::aruco::detectMarkers
    * In camera coordinates
    * */
    cv::Vec3f rVec = 0;
    /**
    * Translation (position) of marker  x (left) ,y (down) ,z (forward)
    * as found by cv::aruco::detectMarkers
    * */
    cv::Vec3f tVec = 0;
    cv::Vec3f test = 0;
    /**
    * Coordinate conversion matrix [4x4] from a position in ball coordinates (left, up, out)
    * to robot coordinates (forward, left, up) */
    cv::Mat ball2Cam;
    /**
    * Ball position in robot coordinates
    * x = forward, y=left, z=up */
    cv::Mat ballPosition;
    // euclidean distance in meters
    float distance2ball = 0;

    float ballAngle = 0;

    std::mutex lock;

    UPose robotPose;

public:
    /* Make ball to camera coordinate conversion matrix based on 
    *  
    */
    cv::Mat makeBallToCam4x4matrix(cv::Vec3f rVec, cv::Vec3f tVec);

    void ballToRobotCoordinate(cv::Mat cam2robot);
};

class UCamera;

class FindBalls
{
public:
    // Pointer to camera info
    UCamera *cam;

    int frameCnt = 0;

    FindBall findBall; // Data class

public:
    FindBalls(UCamera *iCam)
    {
        cam = iCam;
    }

    ~FindBalls();

    static const int MAX_VAL_CNT = 1;

    int doFindBallProcessing(cv::Mat frame, int frameNumber, UTime imTime);
    int doFindBallProcessingCloud(cv::Mat frame, int frameNumber, UTime imTime);

    void makeCamToRobotTransformation();

    inline FindBall *returnDataPointer() { return &findBall; };

    inline double rad2deg(double a) { return a * 180 / M_PI; };
    inline double deg2rad(double a) { return a / 180 * M_PI; };

    void setPoseAtImageTime(float x, float y, float_t h);
};

#endif
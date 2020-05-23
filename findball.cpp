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
#include "cloud_process.h"

using namespace std;

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////// FindBall class ////////////////////
//////////////////////////////////////////////////

cv::Mat FindBall::makeBallToCam4x4matrix(cv::Vec3f rVec, cv::Vec3f tVec)
{
    cv::Mat R, camH;
    Rodrigues(rVec, R);          // 3 cols, 3 rows
    cv::Mat col = cv::Mat(tVec); // 1 cols, 3 rows
    hconcat(R, col, camH);       // 4 cols, 3 rows
    float tempRow[4] = {0, 0, 0, 1};
    cv::Mat row = cv::Mat(1, 4, CV_32F, tempRow); // 4 cols, 1 row
    camH.push_back(row);                          // 4 cols, 4 rows
    return camH;                                  // 4 cols, 4 rows
}

////////////////////////////////////////////////////

void FindBall::ballToRobotCoordinate(cv::Mat cam2robot)
{
    // make ball to camera coordinate conversion matrix for this marker
    ball2Cam = makeBallToCam4x4matrix(rVec, tVec);
    // combine with camera to robot coordinate conversion
    cv::Mat ball2robot = cam2robot * ball2Cam;

    // get position of ball center
    cv::Vec4f zeroVec = {0, 0, 0, 1};
    ballPosition = ball2robot * cv::Mat(zeroVec);
    // get position of 4cm in z-ball direction - out from ball
    cv::Vec4f zeroVecZ = {0, 0, 0.04, 1};
    cv::Mat ball4cmVecZ = ball2robot * cv::Mat(zeroVecZ);
    // get ball z-vector (in robot coordinate system)
    cv::Mat dz4cm = ball4cmVecZ - ballPosition;

    // ASSUMES VERTICAL BALL ?
    // rotation of marker Z vector in robot coordinates around robot Z axis
    ballAngle = atan2(-dz4cm.at<float>(0, 1), -dz4cm.at<float>(0, 0));

    // in plane distance sqrt(x^2 + y^2) only - using hypot(x,y) function
    distance2ball = hypotf(ballPosition.at<float>(0, 0), ballPosition.at<float>(0, 1));
    if (true)
    { // debug
        printf("# Ball found at(%.3fx, %.3fy, %.3fz) (robot coo), plane dist %.3fm, angle %.3f rad, or %.1f deg\n",
               ballPosition.at<float>(0, 0), ballPosition.at<float>(0, 1), ballPosition.at<float>(0, 2),
               distance2ball, ballAngle, ballAngle * 180 / M_PI);
    }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////// FindBalls class ////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

int FindBalls::doFindBallProcessingCloud(cv::Mat frame, int frameNumber, UTime imTime)
{
    CloudProcess proc;
    vector<vector<cv::Point2f>> ballCorners;
    vector<cv::Vec3d> rotationVectors, translationVectors;
    const float ballSqaureDimensions = 0.041;

    cv::imwrite("frame.jpg", frame);

    system("scp frame.jpg dennis@192.168.1.149:~/cloud_process/Images/");
    int listen = 10;
    const char *ip = "localhost"; // MQTT broker connection ip address (192.168.1.149 Dennis computer at martins house)
    ballCorners = proc.wait_for_corners(listen, ip);

    printf("Ball corners:\n");
    for (int i = 0; i < 4; i++)
    {
        printf("(%.2f,%.2f)\t\t", ballCorners[0][i].x, ballCorners[0][i].y);
    }

    cv::aruco::estimatePoseSingleMarkers(ballCorners,
                                         ballSqaureDimensions,
                                         cam->cameraMatrix,
                                         cam->distortionCoefficients,
                                         rotationVectors,
                                         translationVectors);
    // Data class returned owned by FindBalls
    FindBall *v = FindBalls::returnDataPointer();
    v->lock.lock();
    v->imageTime = imTime;
    v->frameNumber = frameNumber;
    v->rVec = rotationVectors[0];
    v->tVec = translationVectors[0];
    v->ballToRobotCoordinate(cam->cam2robot);
    v->lock.unlock();

    return EXIT_SUCCESS;
}

int FindBalls::doFindBallProcessing(cv::Mat frame, int frameNumber, UTime imTime)
{
    const float ballSqaureDimensions = 0.041; //4.1 cm diameter
    vector<vector<cv::Point2f>> ballCorners;
    vector<cv::Point2f> corners;

    vector<cv::Vec3d> rotationVectors, translationVectors;

    UTime t;
    t.now();

    // matrix to store gray picture
    cv::Mat hsv_img;
    cv::Mat thresh;

    vector<cv::Vec3f> circles;
    cv::cvtColor(frame, hsv_img, cv::COLOR_BGR2HSV);
    cv::medianBlur(hsv_img, hsv_img, 25);

    erode(hsv_img, hsv_img, 4);
    dilate(hsv_img, hsv_img, 4);

    inRange(hsv_img, cv::Scalar(7, 35, 78), cv::Scalar(30, 225, 240), thresh);

    cv::HoughCircles(thresh, circles, cv::HOUGH_GRADIENT, 1, thresh.rows / 2, 30, 15, 10, 200);

    if (circles.size() == 1)
    {
        cv::Vec3i c = circles[0];
        int x = floor(c[0]);
        int y = floor(c[1]);
        int r = floor(c[2]);
        printf("Centre coordinates:\t");
        printf("x = %d, y = %d\n", x, y);
        printf("Radius:\t\t\tr = %d pixels\n", r);

        corners.push_back(cv::Point2f(x - r, y - r));
        corners.push_back(cv::Point2f(x + r, y - r));
        corners.push_back(cv::Point2f(x + r, y + r));
        corners.push_back(cv::Point2f(x - r, y + r));

        ballCorners.push_back(corners);

        printf("Ball corners:\n");
        for (int i = 0; i < 4; i++)
        {
            printf("(%.2f,%.2f)\t\t", ballCorners[0][i].x, ballCorners[0][i].y);
        }

        cv::aruco::estimatePoseSingleMarkers(ballCorners,
                                             ballSqaureDimensions,
                                             cam->cameraMatrix,
                                             cam->distortionCoefficients,
                                             rotationVectors,
                                             translationVectors);
        // Data class returned owned by FindBalls
        FindBall *v = FindBalls::returnDataPointer();
        v->lock.lock();
        v->imageTime = imTime;
        v->frameNumber = frameNumber;
        v->rVec = rotationVectors[0];
        v->tVec = translationVectors[0];
        v->ballToRobotCoordinate(cam->cam2robot);
        v->lock.unlock();

        return EXIT_SUCCESS;
    }
    else
    {
        printf("Found more than one circle or none\n");

        return EXIT_FAILURE;
    }
}

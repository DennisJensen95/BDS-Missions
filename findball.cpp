// Find ball class

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <unistd.h>
#include <stdlib.h>
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

    //cout << "ball2robot = " << endl << " "  << ball2robot << endl << endl;

    // get position of ball center
    cv::Vec4f zeroVec = {0, 0, 0, 1};
    ballPosition = ball2robot * cv::Mat(zeroVec);
    // get position of 10cm in z-ball direction - out from ball
    cv::Vec4f zeroVecZ = {0, 0, 0.1, 1};
    cv::Mat ball10cmVecZ = ball2robot * cv::Mat(zeroVecZ);
    // get ball z-vector (in robot coordinate system)
    cv::Mat dz10cm = ball10cmVecZ - ballPosition;

    // ASSUMES VERTICAL BALL ?
    // rotation of marker Z vector in robot coordinates around robot Z axis
    ballAngle = atan2(-dz10cm.at<float>(0, 1), -dz10cm.at<float>(0, 0));
    //ballAngle = -10/(float)180*M_PI;
    //ballAngle = 0;

    // in plane distance sqrt(x^2 + y^2) only - using hypot(x,y) function
    distance2ball = hypotf(ballPosition.at<float>(0, 0), ballPosition.at<float>(0, 1));
    if (true)
    { // debug
        printf("# Ball found at(%.3fx, %.3fy, %.3fz) (robot coo), plane dist %.3fm, angle %.3f rad, or %.1f deg, dz (x, y) values are in deg: (%.3f, %.3f)\n",
               ballPosition.at<float>(0, 0), ballPosition.at<float>(0, 1), ballPosition.at<float>(0, 2),
               distance2ball, ballAngle, ballAngle * (float)180 / M_PI, dz10cm.at<float>(0, 0), dz10cm.at<float>(0, 1));
    }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
////////////// FindBalls class ////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////

int FindBalls::doFindBallProcessingCloud(cv::Mat frame, int frameNumber, UTime imTime)
{
    vector<vector<cv::Point2f>> ballCorners;
    vector<cv::Vec3d> rotationVectors, translationVectors;
    const float ballSqaureDimensions = 0.041;
    printf("Sending image\n");
    cv::imwrite("frame.jpg", frame);

    system("scp frame.jpg dennis@192.168.1.149:~/cloud_process/Images/frame.jpg");
    int listen = 6;
    const char *ip = "192.168.1.149"; // MQTT broker connection ip address (192.168.1.149 Dennis computer at Martins house)
    printf("Waiting for cloud processing\n");
    ballCorners = wait_for_corners(listen, ip);

    printf("Ball corners:\n");
    for (int i = 0; i < 4; i++)
    {
        printf("(%.2f,%.2f)\t\t", ballCorners[0][i].x, ballCorners[0][i].y);
        if (ballCorners[0][i].x == 10 && ballCorners[0][i].y == 10) 
        {
            return EXIT_FAILURE;
        }
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
    bool debug = true;

    const float ballSqaureDimensions = 0.04; //4.1 cm diameter
    vector<vector<cv::Point2f>> ballCorners;
    vector<cv::Point2f> corners;

    vector<cv::Vec3d> rotationVectors, translationVectors;

    UTime t;
    t.now();

    int erosion_size = 11;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
        cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));

    // matrix to store gray picture
    cv::Mat gray,hsv,frame_thresh;

    // vector to store the found circle
    vector<cv::Vec3f> circles;

    // convert to gray scale
    //cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    
    // stretch colour spectrum
    //cv::equalizeHist(gray, gray);

    // blur image for better processing
    //cv::medianBlur(gray, gray, 11);
    cv::medianBlur(hsv, hsv, 11);

    inRange(hsv, cv::Scalar(1, 90, 100), cv::Scalar(20, 230, 240), frame_thresh); //ball 

    if(debug){
        cv::imwrite("thresh.jpg", frame_thresh);
    }

    // erosion removes noise
    cv::erode(frame_thresh, frame_thresh, cv::Mat(), cv::Point(-1, -1), 5); // cv::Mat() gives a 3x3 square structuring element
    /// dilate to create square around ball
    cv::dilate(frame_thresh, frame_thresh, element, cv::Point(-1, -1), 5);

    cv::bitwise_and(frame, frame, gray, frame_thresh);

    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);

    // save image
    if(debug){
        cv::imwrite("gray.jpg", gray);
    }

    // do hough circles
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 10000, 50, 5, 0, 70);

    // save image
    if(debug){
        cv::imwrite("frame.jpg", frame);
    }

    //sleep(1);
    //system("scp frame.jpg dennis@192.168.1.149:~/cloud_process/Images/frame.jpg");


    // draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        cv::circle( frame, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( frame, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

    // save image with circles
    if(debug){
        cv::imwrite("circles.jpg", frame);
    }

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
            printf("\t(%.2f,%.2f)\n\t\t", ballCorners[0][i].x, ballCorners[0][i].y);
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

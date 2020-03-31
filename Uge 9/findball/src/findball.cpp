#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main()
{

    Mat img;
    img = imread("../../ball7.jpg", IMREAD_COLOR); // Read the file
    if (img.empty())                                  // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    Mat gray;

    vector<Vec3f> circles;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 21);

    HoughCircles(gray, circles, HOUGH_GRADIENT, 1, gray.rows / 8, 100, 50, 100,400);

    if (circles.size() == 1){
        Vec3i c = circles[0];
        int x = floor(c[0]);
        int y = floor(c[1]);
        int r = floor(c[2]);

        printf("Centre coordinates:\t");
        printf("x = %d, y = %d\n", x, y);
        printf("Radius:\t\t\tr = %d pixels\n", r);
    }

    return 0;
}

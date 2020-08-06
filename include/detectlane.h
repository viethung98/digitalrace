#ifndef DETECTLANE_H
#define DETECTLANE_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <algorithm>

using namespace std;
using namespace cv;

class DetectLane
{
public:
    DetectLane();
    ~DetectLane();

    void update(const Mat &src);
    
    vector<Point> getLeftLane();
    vector<Point> getRightLane();

    static int slideThickness;

    static int BIRDVIEW_WIDTH;
    static int BIRDVIEW_HEIGHT;

    static int VERTICAL;
    static int HORIZONTAL;

    static Point null; // 

private:
    Mat preProcess(const Mat &src);

    Mat morphological(const Mat &imgHSV);
    Mat birdViewTranform(const Mat &source);
    void fillLane(Mat &src);
    vector<Mat> splitLayer(const Mat &src, int dir = VERTICAL);
    vector<vector<Point> > centerRoadSide(const vector<Mat> &src, int dir = VERTICAL);
    void detectLeftRight(const vector<vector<Point> > &points);
    Mat laneInShadow(const Mat &src);

    // int minThreshold[3] = {0, 0, 192};
    // int maxThreshold[3] = {115, 30, 245};
    // int minShadowTh[3] = {80, 65, 86};
    // int maxShadowTh[3] = {110, 90, 131};

    int minThreshold[3] = {18, 0, 218};
    int maxThreshold[3] = {106, 36, 241};
    int minShadowTh[3] = {105, 76, 70};
    int maxShadowTh[3] = {109, 94, 122};


    int minLaneInShadow[3] = {1, 1, 32};
    int maxLaneInShadow[3] = {210, 199, 171};
    int binaryThreshold = 180;

    int skyLine = 90;
    int shadowParam = 40;

    vector<Point> leftLane, rightLane;
};

#endif

#ifndef CARCONTROL_H
#define CARCONTROL_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include <vector>
#include <math.h>

#include "detectlane.h"

using namespace std;
using namespace cv;

class CarControl 
{
public:
    CarControl();
    ~CarControl();
    void driverCar(const vector<Point> &left, const vector<Point> &right, float velocity, int turn);
    bool isStable(const vector<Point> &left, const vector<Point> &right);
    bool isCross(const vector<Point> &left, const vector<Point> &right);
    bool isTurningLeft, isTurningRight;
    bool prepareLeft, prepareRight;
    int leftParamAdd, rightParamAdd;
    int leftParamDiv, rightParamDiv;
    int leftMinThres, rightMinThres;
    int leftMaxThres, rightMaxThres;
    int currLeftParam, currRightParam;
    int currentParam;
    int foo1;
    bool foo2;
    Point leftPoint, rightPoint;
    
private:
    float errorAngle(const Point &dst);
    ros::NodeHandle node_obj1;
    ros::NodeHandle node_obj2;
    
    ros::Publisher steer_publisher;
    ros::Publisher speed_publisher;

    Point carPos;

    float laneWidth = 40;

    float minVelocity = 10;
    float maxVelocity = 50;

    float preError;

    float kP;
    float kI;
    float kD;

    int t_kP;
    int t_kI;
    int t_kD;

    void turnLeft(float velocity);
    void turnRight(float velocity);
    void goStraight(const vector<Point> &left, const vector<Point> &right, float velocity);

};

#endif

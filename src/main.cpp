#define _DEBUG
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include "detectlane.h"
#include "carcontrol.h"
#include "std_msgs/String.h"
#include <string.h>

bool STREAM = true;

VideoCapture capture("video.avi");
DetectLane *detect;
CarControl *car;
int skipFrame = 1;
int speed = 60;

int turn = 0;

void signCallback(const std_msgs::String::ConstPtr& msg) {
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    const char *signChar = msg->data.c_str();
    if (strcmp(signChar, "") == 0)
        turn = 0;
    else if (strcmp(signChar, "TURN LEFT") == 0)
        turn = -1;
    else if (strcmp(signChar, "TURN RIGHT") == 0)
        turn = 1;
    else
        turn = -2;
    // ROS_INFO("%d", turn);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    Mat out;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
#ifdef _DEBUG
        cv::imshow("View", cv_ptr->image);
#endif
	    waitKey(1);
        detect->update(cv_ptr->image);
        car->driverCar(detect->getLeftLane(), detect->getRightLane(), speed, turn);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void videoProcess()
{
    Mat src;
    while (true)
    {
        capture >> src;
        if (src.empty()) break;
        
        imshow("View", src);
        detect->update(src);
        waitKey(30);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
#ifdef _DEBUG
    cv::namedWindow("View");
    cv::namedWindow("Binary");
    cv::namedWindow("Threshold");
    // cv::namedWindow("Bird View");
    cv::namedWindow("Lane Detect");
    cv::namedWindow("ShadowMask");
    // cv::namedWindow("LaneShadow");
    // cv::namedWindow("CarControl");

    cvCreateTrackbar("Speed", "Threshold", &speed, 255);
#endif

    detect = new DetectLane();
    car = new CarControl();

    if (STREAM) {
        cv::startWindowThread();

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe("team202_image", 1, imageCallback);
        ros::Subscriber sub2 = nh.subscribe("sign", 1000, signCallback);

        ros::spin();
    } else {
        videoProcess();
    }
    cv::destroyAllWindows();
}

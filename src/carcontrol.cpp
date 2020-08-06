#include "carcontrol.h"

CarControl::CarControl()
{
    carPos.x = 120;
    carPos.y = 300;
    steer_publisher = node_obj1.advertise<std_msgs::Float32>("team202_steerAngle",10);
    speed_publisher = node_obj2.advertise<std_msgs::Float32>("team202_speed",10);
    isTurningLeft = false;
    isTurningRight = false;
    prepareLeft = false;
    prepareRight = false;
    leftParamAdd = 1; rightParamAdd = 1;
    leftParamDiv = 1; rightParamDiv = 1;
    leftMinThres = 20; rightMinThres = 1;
    leftMaxThres = 70; rightMaxThres = 70;
    currLeftParam = 0, currRightParam = 0;
    foo1 = 0;
    foo2 = false;

    // cvCreateTrackbar("leftParamAdd", "CarControl", &leftParamAdd, 10000);
    // cvCreateTrackbar("leftParamDiv", "CarControl", &leftParamDiv, 10000);
    // cvCreateTrackbar("leftMinThres", "CarControl", &leftMinThres, 10000);
    // cvCreateTrackbar("leftMaxThres", "CarControl", &leftMaxThres, 10000);

    // cvCreateTrackbar("rightParamAdd", "CarControl", &rightParamAdd, 10000);
    // cvCreateTrackbar("rightParamDiv", "CarControl", &rightParamDiv, 10000);
    // cvCreateTrackbar("rightMinThres", "CarControl", &rightMinThres, 10000);
    // cvCreateTrackbar("rightMaxThres", "CarControl", &rightMaxThres, 10000);
}

CarControl::~CarControl() {}

float CarControl::errorAngle(const Point &dst)
{
    if (dst.x == carPos.x) return 0;
    if (dst.y == carPos.y) return (dst.x < carPos.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - carPos.x;
    double dy = carPos.y - dst.y; 
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void CarControl::goStraight(const vector<Point> &left, const vector<Point> &right, float velocity)
{
    int i = left.size() - 11;
    float error = preError;
    while (left[i] == DetectLane::null && right[i] == DetectLane::null) {
        i--;
        if (i < 0) return;
    }

    Point temp;

    if (left[i] != DetectLane::null && right[i] !=  DetectLane::null)
    {
        error = errorAngle((left[i] + right[i]) / 2);
        temp = (left[i] + right[i]) / 2;
    } 
    else if (left[i] != DetectLane::null)
    {
        error = errorAngle(left[i] + Point(laneWidth / 2, 0));
        temp = (left[i] + Point(laneWidth / 2, 0));
    }
    else
    {
        error = errorAngle(right[i] - Point(laneWidth / 2, 0));
        temp = right[i] - Point(laneWidth / 2, 0);
    }

    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    if (error > 50)
        error = 50;
    angle.data = error;
    speed.data = velocity;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed);    
} 

void CarControl::turnLeft(float velocity)
{
    std::cout << "Turnleft\n";
    float error = errorAngle(Point(20, 200));
    
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    if (velocity > 100)
    {
        error *= 3;
    }

    angle.data = error;
    speed.data = velocity - 10;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed); 
}

void CarControl::turnRight(float velocity)
{
    std::cout << "TurnRight\n";
    float error = errorAngle(Point(240, 150));
    std_msgs::Float32 angle;
    std_msgs::Float32 speed;

    angle.data = error;
    speed.data = velocity - 10;

    steer_publisher.publish(angle);
    speed_publisher.publish(speed); 
}

void CarControl::driverCar(const vector<Point> &left, const vector<Point> &right, float velocity, int turn)
{
    // Turn left
    if (isStable(left, right))
    {
        if (foo2)
        {
            velocity -= 5;
            foo1 += 1;
            // cout << foo1 << endl;
        }
        else
        {
            foo2 = true;
        }
        
    }
    else
    {
        foo2 = false;
        foo1 = 0;
    }
    if (isTurningLeft)
    {
        turnLeft(velocity);
        currLeftParam -= leftParamDiv;
        if (currLeftParam < leftMinThres)
        {
            currLeftParam = 0;
            isTurningLeft = false;
        }
    }
    else if (isTurningRight)
    {
        turnRight(velocity);
        currRightParam -= rightParamDiv;
        if (currRightParam < rightMinThres)
        {
            currRightParam = 0;
            isTurningRight = false;
        }
    }
/*    
    else if (isCross(left, right))
    {
        if (! foo)
        {
            currRightParam = rightMaxThres;
            turnRight(velocity);
            isTurningRight = true;
            foo = true;
        }
        else
        {
            currLeftParam = leftMaxThres;
            turnLeft(velocity);
            isTurningLeft = true;
            foo = false;
        }

    }*/
    else if (isCross(left, right))
    {
        // cout << currRightParam << " " << currLeftParam << endl;
        if (currRightParam > 1)
        {
            isTurningLeft = false;
            isTurningRight = true;
            currRightParam = rightMaxThres;
        }
        else
        {
            if (currLeftParam > leftMinThres)
            {
                isTurningRight = false;
                isTurningLeft = true;
                currLeftParam = leftMaxThres;
            }
            else
            {
                currLeftParam += 1;
            }
            // cout << "a " << currLeftParam << endl;

        }
    }
    else
    {
        if (turn == -1)
        {
            currLeftParam += leftParamAdd;
            // cout << currLeftParam << endl;  

        }
        else if (turn == 1)
        {
            currRightParam += rightParamAdd;

        }
        else
        {
            goStraight(left, right, velocity);
        }
        
    }   
} 

bool CarControl::isStable(const vector<Point> &left, const vector<Point> &right)
{
/*    int nleft = 0, nright = 0, ns = 0;
    for (int i = 0; i < left.size(); i ++)
    {
        if (left[i] == DetectLane::null)
            nleft ++;
        if (right[i] == DetectLane::null)
            nright ++;
        if (left[i] != DetectLane::null && right[i] != DetectLane::null)
        {
            if ((1 > right[i].x - left[i].x) || (70 < right[i].x - left[i].x))
                ns ++;                
        }
    }
    if (nleft < left.size() / 2 || nright < right.size() / 2)
        return false;

    if (ns > 10)
        return false;
    return true;*/

    int threshold = 0, maxdist = 0;

    for (int i = 0; i < left.size(); i ++)
    {
        if (left[i] != DetectLane::null && right[i] != DetectLane::null)
        {
            if (maxdist < right[i].x - left[i].x)
                maxdist = right[i].x - left[i].x;
        }
    }
    return maxdist == 0;
}



bool CarControl::isCross(const vector<Point> &left, const vector<Point> &right)
{
    int threshold = 0, maxdist = 0;

    for (int i = 0; i < left.size(); i ++)
    {
        if (left[i] != DetectLane::null && right[i] != DetectLane::null)
        {
            if (maxdist < right[i].x - left[i].x)
                maxdist = right[i].x - left[i].x;
        }
    }
    // cout << maxdist << endl;
    if (maxdist > 803)
    {
        return true;
    }
    return false;

}
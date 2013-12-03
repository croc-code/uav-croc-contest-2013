/*********************************************************	

(c) CROC Inc 2013

This source code demonstrates navigation, take-off, landing
and visual mark detection algorithms  we’ve developed for
CROC Flying robot competition 2013 (www.robots.croc.ru ).

croc_CalcLandingPos - node that transforms coordinates on
image to physical coordinates on the ground. This physical
coordinates are sets as global goal later in croc_navi.


Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "croc_Pose3D/Pose3D.h"
#include <deque>

using namespace std;


// Quad 3D position
class QuadPos
{
public:
    float x;	
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;

    QuadPos()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        yaw = 0.0;
        pitch = 0.0;
        roll = 0.0;
    }

    QuadPos(float X, float Y, float Z, float Yaw, float Pitch, float Roll)
    {
        x = X;
        y = Y;
        z = Z;
        yaw = Yaw;
        pitch = Pitch;
        roll = Roll;
    }
};

QuadPos quadPosPrev;					// Quad position previous to landing mark detection

deque <geometry_msgs::Pose2D> poses; 	// Detected poses

float xIntersection = -100.0;			// x-coordinate of detected mark on floor (relative to quad starting point)
float yIntersection = -100.0;           // y-coordinate of detected mark on floor (relative to quad starting point)

int camHorRes = 640;					// camera horizontal resolution (pixels)
int camVerRes = 480;					// camera vertical resolution (pixels)
double camHorAngle = 120 * M_PI/180;	// camera horizontal angle 
double camVerAngle = 120 * M_PI/180;	// camera vertical angle 

int opticalCenterX = camHorRes/2; 		// optical center of camera (X)
int opticalCenterY = camVerRes/2; 		// optical center of camera (Y)

int averagePosesCount = 3;      		// number of history poses for landig position calculation


geometry_msgs::Pose2D fromDetectingMark;// coordinates of detected mark


ros::Publisher publisher;				// node result publisher 
ros::Subscriber subscriberPose3D;		// quad current position subscriber
ros::Subscriber subscriberDetectingMark;// coordinates of detected mark subscriber


// calculation of physical coordinates of landing mark
bool calcIntersectionPoint();			

// get node parameters 
void getParameters(const ros::NodeHandle& n);	


// calaculation of average coordinates of landing mark based on history values
// n: number of history values
geometry_msgs::Pose2D getAverageLandingPose(int n);	


// Quad current postion callback
void callbackQuadPos(const croc_Pose3D::Pose3D& QuadPosition)
{
    quadPosPrev.x = QuadPosition.x;
    quadPosPrev.y = QuadPosition.y;
    quadPosPrev.z = QuadPosition.z/100; 
    quadPosPrev.yaw = QuadPosition.yaw + M_PI;  // camera on the back of quad
    quadPosPrev.pitch = QuadPosition.pitch;

}

// Detected mark current coordinates callback
void callbackMarkerPos(const geometry_msgs::Pose2D& MarkerPos)
{

    fromDetectingMark.x = MarkerPos.x;
    fromDetectingMark.y = MarkerPos.y;

    if (calcIntersectionPoint())
    {
        geometry_msgs::Pose2D LandingFieldPos;
      
        LandingFieldPos.x = xIntersection;
        LandingFieldPos.y = yIntersection;

        poses.push_back(LandingFieldPos);
        if (poses.size()>averagePosesCount) poses.pop_front();

        publisher.publish(getAverageLandingPose(averagePosesCount));
    }


}

// calculate average lnding pose based on n previous values;
geometry_msgs::Pose2D getAverageLandingPose(int n)
{
    geometry_msgs::Pose2D averagePose;

    if (poses.size()<n) n=poses.size();

    double x=0;
    double y=0;

    for(int i=0; i<n; i++)
    {
        x=x+poses[i].x;
        y=y+poses[i].y;
    }
    x=x/n;
    y=y/n;

    averagePose.x=x;
    averagePose.y=y;

    return averagePose;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "croc_CalcLandingPos");
    ros::NodeHandle n;

    getParameters(n);

    publisher = n.advertise<geometry_msgs::Pose2D>("LandingFieldPos", 1000);

    subscriberPose3D = n.subscribe("Pose3D", 1000, &callbackQuadPos);
    subscriberDetectingMark = n.subscribe("DetectingMark", 1000, &callbackMarkerPos);

    while (ros::ok())
        {
            ros::WallDuration(0.02).sleep();
            ros::spinOnce();
        }

    return 0;

}


void getParameters(const ros::NodeHandle &n)
{
    //!!! what is screen resolution?

    if(n.getParam("/croc_CalcLandingPos/camHorRes", camHorRes))
    {
        cout << "read camHorRes" << endl;
    }
    else
    {
        cout << "can't read camHorRes, use default" << endl;
    }

    if(n.getParam("/croc_CalcLandingPos/camVerRes", camVerRes))
    {
        cout << "read camVerRes" << endl;
    }
    else
    {
        cout << "can't read camVerRes, use default" << endl;
    }

    if(n.getParam("/croc_CalcLandingPos/camHorAngle", camHorAngle))
    {
        cout << "read camHorAngle" << endl;
    }
    else
    {
        cout << "can't read camHorAngle, use default" << endl;
    }

    if(n.getParam("/croc_CalcLandingPos/camVerAngle", camVerAngle))
    {
        cout << "read camVerAngle" << endl;
    }
    else
    {
        cout << "can't read camVerAngle, use default" << endl;
    }

    if(n.getParam("/croc_CalcLandingPos/opticalCenterX", opticalCenterX))
    {
        cout << "read opticalCenterX" << endl;
    }
    else
    {
        cout << "can't read opticalCenterX, use default" << endl;
    }

    if(n.getParam("/croc_CalcLandingPos/opticalCenterY", opticalCenterY))
    {
        cout << "read opticalCenterY" << endl;
    }
    else
    {
        cout << "can't read opticalCenterY, use default" << endl;
    }

    if(n.getParam("/croc_CalcLandingPos/averagePosesCount", averagePosesCount))
    {
        cout << "read averagePosesCount" << endl;
    }
    else
    {
        cout << "can't read averagePosesCount, use default" << endl;
    }

}


bool calcIntersectionPoint()
{

	// angles for physical landing position calculation
	float alpha = (fromDetectingMark.x - opticalCenterX) * camHorAngle/camHorRes;
    float beta = (fromDetectingMark.y - opticalCenterY) * camVerAngle/camVerRes;
    double vertAngle = beta - quadPosPrev.pitch;

    if (std::abs(vertAngle)>0.001)
    {
		// Simple method
		
        float a = quadPosPrev.z * 1/tan(beta - quadPosPrev.pitch);	// distance to landing mark
        if (a>0)
        {
            xIntersection = quadPosPrev.x + a * cos(- alpha + quadPosPrev.yaw);
            yIntersection = quadPosPrev.y + a * sin(- alpha + quadPosPrev.yaw);
        }
        else
        {
            return false;
        }

    }
    else
    {
        return false;
    }

    return true;
}



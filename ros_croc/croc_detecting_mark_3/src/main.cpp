/*********************************************************	

(c) CROC Inc 2013

This source code demonstrates navigation, take-off, landing
and visual mark detection algorithms  we’ve developed for
CROC Flying robot competition 2013 (www.robots.croc.ru ).

croc_detecting_mark_3 - node with OpenCV implementation of
landing mark detection. When landmark is found, node
publishes coordinates of landing mark center on image.

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include "stdio.h"
#include "math.h"


#include "croc_detecting_mark/ros_opencv_bridge.h"
#include "geometry_msgs/Pose2D.h"
#include "croc_detecting_mark/detecting_mark.h"

#include <boost/lexical_cast.hpp>

#include "ros/ros.h"
#include "std_msgs/Int32.h"

using namespace cv;
using namespace std;

/// init and start ROS node
int main (int argc, char *argv[])
{
    // Set name of node
    ros::init(argc, argv, "DetectingMark3");

    // Create ros::NodeHandle after ros::init to interact with ROS
    ros::NodeHandle nn("~");

    // Create instance, init with ROS node handle
    DetectingMark DM(nn);

    // Launch node instance, catch exceptions
    try
    {
      DM.run();
    }
    catch (exception& e)
    {
        cout << "DetectingMark3::run() exception: " <<  e.what() << endl;
      return -1;
    }

    return 0;
}


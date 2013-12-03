#ifndef DETECTING_MARK_H
#define DETECTING_MARK_H

#include "opencv/cv.h"

#include "opencv2/opencv.hpp" // FileStorage
#include "opencv2/highgui/highgui.hpp" // cvCreateFileCapture'

#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32.h" // for callbackStopPause
#include "croc_detecting_mark/ros_opencv_bridge.h"
#include "ros/ros.h"

#include"math.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "croc_detecting_mark_3/HSVrange.h"

using namespace std;
using namespace cv;
using namespace ros;

#define TEST

/// Debug log level values
#define LOG_LEVEL_ERROR  0
#define LOG_LEVEL_WARN   1
#define LOG_LEVEL_INFO   2
#define LOG_LEVEL_DEBUG  3

/// Default cycle rate
#define CYCLE_RATE       10


/// enum for choosing video source
enum InputStream { VIDEO, CAMERA, TOPIC };


/// Class that finds landing mark for CROC competition 2013
class DetectingMark
{
public:
    /// Constructor takes as parameter NodeHandler to initialise NodeHandler inside class
    DetectingMark(NodeHandle &n):
        nh_(n)
    {

        mLogLevel_ = LOG_LEVEL_INFO;
        mRate_ = new ros::Rate(CYCLE_RATE);

        /// 1) Read parameters from launch file
        getParams();

        /// 2) Initialse everything according to the parameters that was readen
        initialize();
    }

    ~DetectingMark()
    {
        /// Destroy all windows that used to show different images
        cv::destroyAllWindows();
        /// Destructor for ImageConverter
        m_pImgConverter_->~ImageConverter();
    }

    void run(); /// Function with main loop algorithm. It should be called after getParams & initialize



private:
    Mat mDistCoeffs_;   /// Parameter to undistort image
    Mat mCamMatrix_;    /// Parameter to undistort image
    Mat mInputFrame_;   //! Read frame from input stream and save until the end of the loop
    Mat mFrameToWork_;  //! Copy mInputFrame to mFrameToWork. And algorithms work with mFrameToWork
    Mat mImgToShow1;    // Temp image
    Mat mImgToShow2;    // Temp image

    int mThresh_;       //! Threshold for ellipse detecting
    int mMaxThresh_;    //! Max threshold for ellipse detecting

    float mWhiteRatio_; //! To check the number of white pixels in ellipse

    bool mIsEllipseDetected; /// flag, means ellipse is detected

    bool mWait_; //! Pause algorithm

    bool mStop_; //! Stop algorithm (end of file)

    /// Parameters to read from launcher
    int mFrameWidth_;       // source image width
    int mFrameHeight_;      // source image height
    string mVideoTopic_; /// variable that contain videotopic name
    InputStream mInputStream_; /// variable to choose input stream (videofile, camera, videotopic)
    int mInpStream_; //! To choose from where to read
    string mVideoFile_;  /// variable that contain path + name of videofile
    bool mCorrectImage_; //! This variable say to us correct or not image
    string mPathToCalibYaml_;  /// variable that contain path + name of YAML file with camera & undistortion coefficients
    bool mUseROI_; /// =true if algorithm will use region of interest on frame
    int mROIx_; /// coord X of upper left angle of region of interest
    int mROIy_; /// coord Y of upper left angle of region of interest
    int mROIw_; /// width of region of interest
    int mROIh_; /// height of region of interest
    bool mSourceIsAvailable_; //! =true if we can get frame from VideoInput or ImageConverter
    int mCannyThreshold_;    /// threshold for Canny function
    int mCannyMaxThreshold_;/// Max threshold for Canny function
    int mAlgoMode_; /// mode of algorithm. 0 - publish only precise point, 1 - publish ellipse center first and precise after; 0 is default;

    bool mShowVideo_;               /// if allowed to show video windows
    int mLogLevel_;                 /// level of messages to include in log
                                    /// initialize it so logging starts working
    int mCycleRate_;                /// node cycle rate in Hz

    // HSV ranges
    int mHmax_, mHmin_;
    int mSmax_, mSmin_;
    int mVmax_, mVmin_;

    bool mUseHsv_;            /// if use HSV thresholding instead of intensity

    bool mSendPreTarget_;     /// if need to send picture of found "white ellipse"

    /// Parameters to read from launcher is ended

    // Cycle rate mgr, initialized with default value prior to parameter read
    ros::Rate *mRate_;// = ros::Rate(CYCLE_RATE);

    bool mGetNewImage_; /// = true if we get new image froum source

    VideoCapture mCapture_; /// variable to work with input stream from videofile or camera

    /// Variables to work with robot operating system
    NodeHandle nh_;
    ros::Publisher publisher;
    ros::Subscriber subscribeToPauseStopCmd; /// To subscribe to topic that can pause or stop()exit algorithm
    ros::Subscriber subscribeToHsvRangeCmd;  /// To subscribe to topic with HSV ranges 
    ImageConverter* m_pImgConverter_;  /// Variable to work with video topic

    /// Image publishers
    // Source image (from camera/topic)
    image_transport::Publisher imgSourcePub;
    // Source image with detection results overlaid
    image_transport::Publisher imgDetectionPub;
    // Found intersections
    image_transport::Publisher imgIntersectionPub;
    // Intencity thresholded image
    image_transport::Publisher imgThresholdPub;
    // Found ellipse with target mark
    image_transport::Publisher imgTargetEllipsePub;
    // HSV thresholded image
    image_transport::Publisher imgHSVThresholdPub;
    // Transport for image publishing
    image_transport::ImageTransport* itnode;

    // Alg that publish coordinates of marker on frame if we have several correct ellipses
    geometry_msgs::Pose2D toPublish;
    geometry_msgs::Pose2D toPublishPrecisePoint;
    RotatedRect mEllipseToRoi_;

    // == true if we have intersection inside ellipse (to avoid detecting of bright areas)
    bool mHaveIntersection_;
    // = true if we have new coordinate to publish
    bool mNewPublishInfo;

    // minimum angle diff. betw. detected lines to count them usefull
    int mAngleDiff_;

    /// Variables to work with robot operating system is ended

    // gets parameter value, either from Param Server, or from predefined default
    // various functions for various param datatypes
    // [in] paramPath - path to value
    // [in] defaultVal - default value
    // [retva] - value of parameter
    int getIntParam(std::string paramPath, int defaultVal);
    bool getBoolParam(string paramPath, bool defaultVal);
    string getStrParam(string paramPath, string defaultVal);

    //! Read all parameters from launch file
    void getParams();

    // member variables initializer
    void initialize();

    //! Get frame from source according to configured mInputStream_
    Mat getFrame();

    /// Use this function for correct distortion from camera (you should calibrate
    /// camera before & save distortion coefficient, camera matrix. And read this coeeficients into mDistCoeffs_,
    /// mCamMatrix_).
    void correctDistortion();

    /// Read from Yaml
    void readFromYaml();

    /// return true if ellipse have "normal" size
    bool checkEllipseSize(const RotatedRect box, int frameHeight, int frameWidth);

    // check if enougth white pixels inside given region
    bool checkWhiteInEllipse(const RotatedRect ellipse, Mat threshold_output);

    // draws a coloured ellipse on image with coords inside ROI
    void drawEllipse(cv::Mat frame, RotatedRect ellipseToDraw, cv::Scalar color, bool addROI); // draw ellipse considering ROI

    // ROS event handler - manage start|stop|pause control messages
    void callbackStopPause(const ::std_msgs::Int32& msg);

    // ROS event handler - adjust HSV range values
    void callbackSetHsvRange(const croc_detecting_mark_3::HSVrange& msg);

    // overloaded ROS run cycle handler
    void rosSpinOnce()
    {
        /// ros::spinOnce();
        /// If function catch some exception, program prints text of exception and exit program
        try
        {
            ros::spinOnce();
            // pause until next cycle
            mRate_->sleep();
        }
        catch (ros::Exception& e)
        {
          ROS_ERROR("spinOnce exception: %s", e.what());
          return;
        }
    }

    void cropEllipse_Check(geometry_msgs::Pose2D ellCoord, RotatedRect Ellipse);

    void findStraightLines(const Mat& img);

    void Probabilistic_Hough( Mat src);    

    void findLinesIntersections(vector<Vec4i> lines, Mat srcToDraw);

    bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,
                          Point2f &r);

    bool medianFilterForPoints(vector<Vec2i> points);

    static Scalar randomColor(RNG& rng )
    {
        ;//( 0xFFFFFFFF );

        int icolor = (unsigned) rng;
        return Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
    }

    // Writes text to output (log), with module header
    // If messageLevel > logLevel - it won't output (i.e. not enougth significant)
    // [in] messageText - text to output
    // [in] msgLogLevel - logLevel of current message
    void write_log(string messageText, int msgLogLevel);

    // send picture to ROS topic
    static void publishPicture(Mat image, image_transport::Publisher imagePublisher, string imgType);
};

#endif //DETECTING_MARK_H

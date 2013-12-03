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

#include "croc_detecting_mark/detecting_mark.h"

/// defaults for config file params
/// they will be used if no value is read from config
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define VIDEO_TOPIC_NAME "videoTopic"
#define VIDEO_FILE_NAME "capture.mpg"
#define THRESH 150
#define MAX_THRESH 255
#define CORRECT_IMAGE_BOOL false
#define PATH_TO_CALIB_YAML "out_camera_data.yml"
#define USE_ROI_BOOL true
#define ROI_X 0
#define ROI_Y 240
#define ROI_W 640
#define ROI_H 240
#define ANGLE_DIFFERENCE 20.
#define CANNY_THRESH 50
#define CANNY_MAX_THRESH 255
// log level - Error
#define LOG_LEVEL 3
// no video windows
#define SHOW_WINDOW false
#define USE_HSV false
#define SEND_PRE_TARGET false

// gets parameter value, either from Param Server, or from predefined default
// [in] paramPath - path to value
// [in] defaultVal - default value
// [retva] - value of parameter
int DetectingMark::getIntParam(string paramPath, int defaultVal) {
    // return value
    int retVal;

    // try to read from param server
    if(nh_.getParam(paramPath, retVal)) {
        write_log("read '" + paramPath + "' value", LOG_LEVEL_INFO);
    }
    else {
        write_log("can't read '" + paramPath + "', use default", LOG_LEVEL_INFO);
        retVal = defaultVal;
    }

    return retVal;
}

// gets parameter value, either from Param Server, or from predefined default
// [in] paramPath - path to value
// [in] defaultVal - default value
// [retva] - value of parameter
bool DetectingMark::getBoolParam(string paramPath, bool defaultVal) {
    // return value
    bool retVal;

    // try to read from param server
    if(nh_.getParam(paramPath, retVal)) {
        write_log("read '" + paramPath + "' value", LOG_LEVEL_INFO);
    }
    else {
        write_log("can't read '" + paramPath + "', use default", LOG_LEVEL_INFO);
        retVal = defaultVal;
    }

    return retVal;
}

// gets parameter value, either from Param Server, or from predefined default
// [in] paramPath - path to value
// [in] defaultVal - default value
// [retva] - value of parameter
string DetectingMark::getStrParam(string paramPath, string defaultVal) {
    // return value
    string retVal;

    // try to read from param server
    if(nh_.getParam(paramPath, retVal)) {
        write_log("read '" + paramPath + "' value", LOG_LEVEL_INFO);
    }
    else {
        write_log("can't read '" + paramPath + "', use default", LOG_LEVEL_INFO);
        retVal = defaultVal;
    }

    return retVal;
}

//! Read all parameters from launch file
void DetectingMark::getParams() {
    // log level - level of messages to pass to log output
    mLogLevel_ = getIntParam("logLevel", LOG_LEVEL);

    // Enable or not video control windows
    mShowVideo_ = getBoolParam("showVideo", SHOW_WINDOW);

    // main cycle min duration, msec
    mCycleRate_ = getIntParam("cycleRate", CYCLE_RATE);

    // flag to use HSV thresholding
    mUseHsv_ = getBoolParam("useHSV", USE_HSV);

    // flag to send temp image in topic
    mSendPreTarget_ = getBoolParam("SendPreTarget", SEND_PRE_TARGET);

    // HSV range
    mHmax_ = getIntParam("Hmax", 180);
    mHmin_ = getIntParam("Hmin", 0);
    mSmax_ = getIntParam("Smax", 255);
    mSmin_ = getIntParam("Smin", 0);
    mVmax_ = getIntParam("Vmax", 255);
    mVmin_ = getIntParam("Vmin", 0);

    // source image dimensions
    mFrameWidth_ = getIntParam("frameWidth", FRAME_WIDTH);
    mFrameHeight_ = getIntParam("frameHeight", FRAME_HEIGHT);

    mVideoTopic_ = getStrParam("videoTopic", VIDEO_TOPIC_NAME);

    // input image source
    if(nh_.getParam("inputStream", mInpStream_))
    {
        cout << "read inputStream" << endl;

        // from video file
        if( mInpStream_ == 0 )
        {
            mInputStream_ = VIDEO;
        }
        // from camera
        else if( mInpStream_ == 1 )
        {
            mInputStream_ = CAMERA;
        }
        // from ROS topic
        else if( mInpStream_ == 2 )
        {
            cout << "inputStream = TOPIC" << endl;
            mInputStream_ = TOPIC;
        }
    }
    else
    {
        cout << "can't read inputStream, use default" << endl;
        mInputStream_ = VIDEO;
    }

    mVideoFile_ = getStrParam("videoFile", VIDEO_FILE_NAME);

    // Binarization thresholds
    mThresh_ = getIntParam("thresh", THRESH);
    mMaxThresh_ = getIntParam("max_thresh", MAX_THRESH);

    // flag if distortion correction needed
    mCorrectImage_ = getBoolParam("correctImage", CORRECT_IMAGE_BOOL);

    // path co calibration file
    mPathToCalibYaml_ = getStrParam("pathToCalibYaml", PATH_TO_CALIB_YAML);

    // flag if work on ROI of image
    mUseROI_ = getBoolParam("useROI", USE_ROI_BOOL);

    // if using ROI - read it corners
    if( mUseROI_ )
    {
        mROIx_ = getIntParam("ROIx", ROI_X);
        mROIy_ = getIntParam("ROIy", ROI_Y);
        mROIw_ = getIntParam("ROIw", ROI_W);
        mROIh_ = getIntParam("ROIh", ROI_H);
    }

    // angle for lines intersection detection
    mAngleDiff_ = getIntParam("AngleDiff", ANGLE_DIFFERENCE);


    // thresholds for Canny detector
    mCannyThreshold_ = getIntParam("CannyThreshold", CANNY_THRESH);
    mCannyMaxThreshold_ = getIntParam("CannyMaxThreshold", CANNY_MAX_THRESH);

    // result publishing mode
    mAlgoMode_ = getIntParam("AlgoMode", 0);
}


//! Read distortion coeffs from YAML
void DetectingMark::readFromYaml()
{

    FileStorage fs( mPathToCalibYaml_, FileStorage::READ );
    if(!fs.isOpened())
    {
        std::cout << "File is not opened" << std::endl;
        return;
    }
    else
    {
        fs["Distortion_Coefficients"] >> mDistCoeffs_ ;

        fs["Camera_Matrix"] >> mCamMatrix_ ;

#ifdef TEST
        std::cout << "Read: " << std::endl;
        std::cout << "Camera Matrix:" << std::endl;
        std::cout << mCamMatrix_ << std::endl;
        std::cout << "Distortion_Coefficients:" << std::endl;
        std::cout << mDistCoeffs_ << std::endl;
#endif //TEST
    }

}


//! Initialise everything before loop (subscribers/publishers/etc.)
void DetectingMark::initialize() {
    //initialise work variables
    mNewPublishInfo = false;
    mStop_ = false;
    mWait_ = false;
    mGetNewImage_ = false;
    mSourceIsAvailable_ = true;
    mIsEllipseDetected = false;

    // init distortion correction only if needed
    if(mCorrectImage_) {
        readFromYaml();
    }

    // cycle mgr reinitialize with value from parameter
    mRate_ = new ros::Rate(mCycleRate_);

    // init source of frames depending on configured type
    switch(mInputStream_) {
        case CAMERA:
            mCapture_.open(CV_CAP_ANY);
            if(mCapture_.isOpened()) {
                //! Setup camera picture resolution
                mCapture_.set(CV_CAP_PROP_FRAME_WIDTH, mFrameWidth_);//640);l
                mCapture_.set(CV_CAP_PROP_FRAME_HEIGHT, mFrameHeight_);//480);
            }
            else {
                cout << "Camera isn't available" << endl;
                mSourceIsAvailable_ = false;
            }
            break;

        case VIDEO:
            mSourceIsAvailable_ = mCapture_.open(mVideoFile_);
            if (mCapture_.isOpened() == false) {
                cout << "Video file isn't available. Check path & name of file." << endl;
                mSourceIsAvailable_ = false;
            }
            break;

        case TOPIC:
            cout << "ImageConverter(nh_, mVideoTopic_)" << endl;
            m_pImgConverter_ = new ImageConverter(nh_, mVideoTopic_);
            break;
    }

    /// setup ROS topic publishing
    publisher = nh_.advertise<geometry_msgs::Pose2D>("/DetectingMark", 1000);

    /// Image publishing transport with topics
    itnode = new image_transport::ImageTransport(nh_);
    // Source picture with ellipses
    imgDetectionPub = itnode->advertise("/DetectingImage", 1, false);
    // Found intersections
    imgIntersectionPub = itnode->advertise("/IntersectionImage", 1, false);
    // Intensity binarization
    imgThresholdPub = itnode->advertise("/ThresholdImage", 1, false);
    // Found ellipse with target
    imgTargetEllipsePub = itnode->advertise("/TargetEllipseImage", 1, false);
    // Raw source image
    imgSourcePub = itnode->advertise("/SourceImage", 1, false);
    // HSV binarized image
    imgHSVThresholdPub = itnode->advertise("/HSVThresholdImage", 1, false);

    // Messages to stop/pause processing
    subscribeToPauseStopCmd = nh_.subscribe("/LandingFieldDetectionAlgorithmStartStop",1, &DetectingMark::callbackStopPause, this);

    // Messages to change HSV range borders
    subscribeToHsvRangeCmd = nh_.subscribe("/DetectingMark/SetHsvRange", 100, &DetectingMark::callbackSetHsvRange, this);
}


//! Get next frame from source
//  also sets mGetNewImage_ if a new frame was read
Mat DetectingMark::getFrame() {
    // the next frame to return
    Mat temp;

    // depending on frame source
    switch(mInputStream_) {
        // from camera & videofile - same way
        case CAMERA:
        case VIDEO:
            if(mCapture_.grab()) {
                mCapture_ >> temp;
                mGetNewImage_ = true;
            }
            else
                mGetNewImage_ = false;

            break;

        // from ROS topic
        case TOPIC:
            if(m_pImgConverter_->isImageUpdate()) {
                //! Wait for frame from topic
                //lcout << "Wait for frame from topic" << endl;
                temp = m_pImgConverter_->getImage();
                mGetNewImage_ = true;
            }
            else
                mGetNewImage_ = false;

            break;
    }

    return temp;
}


// Correct image distortion
// neds to have matrices set up previously
// changes (overwrites) member mFrameToWork_ with prev. frame image 
void DetectingMark::correctDistortion() {
    // undistorted image
    Mat temp;

    undistort(mFrameToWork_, temp, mCamMatrix_, mDistCoeffs_);

    // overwrite source image
    mFrameToWork_ = temp.clone();
}


// Writes text to output (log), with module header
// If messageLevel > logLevel - it won't output (i.e. not enougth significant)
// [in] messageText - text to output
// [in] msgLogLevel - logLevel of current message
void DetectingMark::write_log(string messageText, int msgLogLevel) {
    if(msgLogLevel <= mLogLevel_)
       std::cout << "[DetectingMark] " << messageText << std::endl;
}


// send picture to ROS topic
// [in] imageToPublish - image to publish
// [in] imagePublisher - ROS topic image publisher
// [in] imgType        - type of data in image (color = 'brg8', greyscale = 'mono8')
void DetectingMark::publishPicture(Mat imageToPublish, image_transport::Publisher imagePublisher, string imgType) {
    // Image to Publish 
    cv_bridge::CvImage cv_img;

    // populate image properties
    cv_img.image = imageToPublish;
    cv_img.encoding = imgType;
    cv_img.header.stamp = ros::Time::now();

    // publish to ros
    imagePublisher.publish(cv_img.toImageMsg());
}


// main work cycle here
void DetectingMark::run() {

    std::cout << "Main loop begin" << std::endl;

    /// Check if source is available
    if( !mSourceIsAvailable_ ) /// mSourceIsAvailable_ changed in initialise();
    {
        std::cout << "Source isn't available" << std::endl;
        return;
    }

    /// Algorithm work until
    while(!mStop_ && ros::ok()){

        /// Check should we wait or not
        /// if mWait_==true, algorithm do nothing except rosSpinOnce
        while(mWait_ && ros::ok())
        {
            /// Call ros::spinOnce() & check exceptions
            rosSpinOnce();
        }

        /// Call ros::spinOnce() & check exceptions
        rosSpinOnce();

        /// Get frame from source (camera, vido file or video topic)
        /// and change mGetNewImage_ to true if we get new frame
        mInputFrame_ = getFrame();

        /// Check if we get new frame (mGetNewImage_==true)
        /// and that frame isn't empty (mInputFrame_.empty()==false)
        if( mGetNewImage_==true && mInputFrame_.empty()==false ) {
            // store original image
            mImgToShow1 = mInputFrame_.clone();

            // publish source frame image
            publishPicture(mInputFrame_, imgSourcePub, "bgr8");

            //! Main algorithm

            /// If we use region of interest (mUseROI_ == true), we clone just mInputFrame_ ROI  to mFrameToWork_
            if(mUseROI_ == true) {
                /// Check if ROI is OK. If ROI is not OK, print about it & exit program

                bool checkRoiWidth = (mROIx_ + mROIw_) <= mInputFrame_.size().width;
                bool checkRoiHeight = (mROIy_ + mROIh_) <= mInputFrame_.size().height;

                if( checkRoiWidth && checkRoiHeight) {
                    Mat temp(mInputFrame_, cv::Rect(mROIx_,mROIy_,mROIw_,mROIh_));
                    mFrameToWork_= temp.clone();
                }
                else {
                    std::cout << "Error with ROI" << std::endl;
                    return;
                }
            }
            else {
                // no ROI set up, use the whole image
                mFrameToWork_ = mInputFrame_.clone();
            }

            /// If mCorrectImage_ == true algoruthm undistort image according to mCamMatrix_, mDistCoeffs_
            if(mCorrectImage_ == true)
            {
                Mat temp;
                undistort(mFrameToWork_, temp, mCamMatrix_, mDistCoeffs_);

                mFrameToWork_ = temp.clone();
            }

            // Coloured working frame
            Mat colourFrame = mFrameToWork_.clone();

            /// Convert RGB to gray
            Mat grayscale;

            cvtColor( mFrameToWork_, grayscale, COLOR_BGR2GRAY );

            mFrameToWork_ = grayscale.clone();

            //imshow("mFrameToWork_", mFrameToWork_);

            // HSV colour space
            Mat hsvFrame, hsvTheshd;
            /// ------------- Ellipse detecting
            {
                Mat threshold_output;
                vector<vector<Point> > contours;
                vector<Vec4i> hierarchy;

                /// Use threshold binarization
                threshold( mFrameToWork_, threshold_output, mThresh_, mMaxThresh_, THRESH_BINARY );

                mImgToShow2 = threshold_output.clone();

                /// clone of the threshold_output will use checkWhiteInEllipse
                Mat temp_threshold_output = threshold_output.clone();

                // use HSV thresholding if necessary
                if(mUseHsv_) {
                    cvtColor( colourFrame, hsvFrame, CV_BGR2HSV );

                    // threshold using HSV
                    inRange(hsvFrame, Scalar(mHmin_, mSmin_, mVmin_), Scalar(mHmax_, mSmax_, mVmax_), hsvTheshd);

                    publishPicture(hsvTheshd, imgHSVThresholdPub, "mono8");

                    if(mShowVideo_) {
                        imshow("HSV Thres", hsvTheshd);
                    }
                    // further use HSV thresholded image
                    threshold_output = hsvTheshd;
                }

#ifdef TEST
                std::string nameWindow = "threshold_output height: " +
                        boost::lexical_cast<std::string>(threshold_output.size().height) +
                        " width: " +
                        boost::lexical_cast<std::string>(threshold_output.size().width) ;
                imshow(nameWindow.c_str(), threshold_output);
#endif //TEST

                /// Find contours
                findContours( threshold_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

                //////
                /// Find the rotated rectangles and ellipses for each contour
                //////
                vector<RotatedRect> minEllipse( contours.size() );
                vector<RotatedRect> ellipseWithCorrectSize;
                vector<RotatedRect> ellipseWithCorrectSizeWhite;

                /// Find all ellipses
                for( size_t i = 0; i < contours.size(); i++ )
                    if( contours[i].size() > 15 )
                        minEllipse[i] = fitEllipse( Mat(contours[i]) );

                /// Check sizes of ellipses & compose vector ellipseWithCorrectSize from ellipse with correct size

                /// Go through all ellipses
                for( size_t i = 0; i < minEllipse.size(); i++ ) {
                    /// Draw all ellipses (yellow color)
                    Scalar yellow_color = Scalar( 0, 255, 255 );
                    drawEllipse(mImgToShow1, minEllipse[i], yellow_color, mUseROI_);

                    /// Check ellipse size. If sizes is correct, add ellipse to vector ellipseWithCorrectSize
                    if(checkEllipseSize(minEllipse[i], mFrameToWork_.size().height, mFrameToWork_.size().width))
                        ellipseWithCorrectSize.push_back(minEllipse[i]);

                }

                if(ellipseWithCorrectSize.size() > 0) {
                    stringstream msg;
                    msg << "ellipses with correct size count " << ellipseWithCorrectSize.size(); 
                    write_log(msg.str(), LOG_LEVEL_DEBUG);
                }

                /// Draw all ellipses with correct size
                for(int i =0; i < (int)ellipseWithCorrectSize.size(); i++) {
                    /// Use pink color to draw ellipse with correct size
                    Scalar pink_color = Scalar( 244, 0, 255 );
                    drawEllipse(mImgToShow1, ellipseWithCorrectSize[i], pink_color, true);
                }

                // Check percentage of white pixel inside ellipses
                for(unsigned int i = 0; i < ellipseWithCorrectSize.size() ; i++) {
                    if(checkWhiteInEllipse(ellipseWithCorrectSize.at(i), temp_threshold_output))
                        ellipseWithCorrectSizeWhite.push_back(ellipseWithCorrectSize.at(i));
                }

                /// Draw correct ellipses (size+color)
                for(unsigned int i = 0; i < ellipseWithCorrectSizeWhite.size(); ++i) {
                    /// Use red to draw correct ellipses
                    Scalar red_color = Scalar( 0, 0, 255 );
                    drawEllipse(mImgToShow1, ellipseWithCorrectSizeWhite[i], red_color, mUseROI_);
                }


                /// Check if we have more than 1 correct ellipse
                if(ellipseWithCorrectSizeWhite.size() > 1) {
                    /// If we have several ellipses, check that they have the same center. In this case publish this center
                    write_log("several ellipses found", LOG_LEVEL_INFO);

                    int countSameCenter = 0;

                    for(int i = 1; i < (int)ellipseWithCorrectSizeWhite.size(); ++i) {
                        if(ellipseWithCorrectSizeWhite[i].center == ellipseWithCorrectSizeWhite[i-1].center)
                            countSameCenter++;
                    }

                    // if every center have the same coordinate
                    if(countSameCenter == (ellipseWithCorrectSizeWhite.size() - 1) ) {
                        write_log("same several ellipses", LOG_LEVEL_INFO);

                        /// If we use ROI than we should correct coordinates to publish
                        if(mUseROI_) {
                            toPublish.x = (int) ellipseWithCorrectSizeWhite[0].center.x + mROIx_;
                            toPublish.y = (int) ellipseWithCorrectSizeWhite[0].center.y + mROIy_;
                        } else {
                            toPublish.x = (int) ellipseWithCorrectSizeWhite[0].center.x;
                            toPublish.y = (int) ellipseWithCorrectSizeWhite[0].center.y;
                        }

                        RotatedRect purp;
                        purp.center.x = toPublish.x;
                        purp.center.y = toPublish.y;
                        purp.size.height = 20;
                        purp.size.width = 20;
                        drawEllipse(mImgToShow1, purp, Scalar(0,255,0), false);///!!!  addROI should be false here

                        if (!mIsEllipseDetected) publisher.publish(toPublish);

                        mIsEllipseDetected = true;

                        mHaveIntersection_ = false;

                        cropEllipse_Check(toPublish, ellipseWithCorrectSizeWhite[0]);

//                        if(mHaveIntersection_ == true)
//                        {
//                            publisher.publish(toPublish);
//                            cout << "Publish X: " << toPublish.x << "; Y: " << toPublish.y << endl;
//                        }
                    }
                    else {
                        /// If correct ellipses have different centers, we publish nothing
                        stringstream msg;
                        msg << "not the same several ellipses ";
                        msg << countSameCenter;
                        msg << " ";
                        msg << ellipseWithCorrectSizeWhite.size();
                        write_log( msg.str(), LOG_LEVEL_INFO);
                    }
                }
                /// If we have only 1 ellipse, we publish it's coordinates
                else if( ellipseWithCorrectSizeWhite.size() == 1 ) {
                    /// If we use ROI than we should correct coordinates to publish
                    if(mUseROI_) {
                        toPublish.x = (int) ellipseWithCorrectSizeWhite[0].center.x + mROIx_;
                        toPublish.y = (int) ellipseWithCorrectSizeWhite[0].center.y + mROIy_;
                    } else {
                        toPublish.x = (int) ellipseWithCorrectSizeWhite[0].center.x;
                        toPublish.y = (int) ellipseWithCorrectSizeWhite[0].center.y;
                    }

                    RotatedRect purp;
                    purp.center.x = toPublish.x;
                    purp.center.y = toPublish.y;
                    purp.size.height = 20;
                    purp.size.width = 20;
                    drawEllipse(mImgToShow1, purp, Scalar(0,255,0), false); ///!!!  addROI should be false here

                    if (!mIsEllipseDetected && mAlgoMode_==1) publisher.publish(toPublish);

                    mIsEllipseDetected = true;

                    mHaveIntersection_ = false;

                    mEllipseToRoi_ = ellipseWithCorrectSizeWhite[0];
//                    cout << " X " << mEllipseToRoi_.center.x << "; Y " << mEllipseToRoi_.center.y<< endl;
//                    cout << " X " << ellipseWithCorrectSizeWhite[0].center.x << "; Y " << ellipseWithCorrectSizeWhite[0].center.y<< endl;
                    cropEllipse_Check(toPublish, mEllipseToRoi_);

//                    if(mHaveIntersection_ == true)
//                    {
//                        publisher.publish(toPublish);
//                        cout << "Publish X: " << toPublish.x << "; Y: " << toPublish.y << endl;
//                    }
                }

                if(mNewPublishInfo) {
                    ellipse(mImgToShow1, RotatedRect(Point(toPublishPrecisePoint.x, toPublishPrecisePoint.y), Size(10,10),0),Scalar(255,255,0),5);
                    publisher.publish(toPublishPrecisePoint);
                    cout << "precise X: " << toPublishPrecisePoint.x << "; Y: " << toPublishPrecisePoint.y << endl;
                    mNewPublishInfo = false;
                }


                // Show images
                if(mShowVideo_) {
                    imshow("From stream", mImgToShow1);
                    imshow("Work Image", mImgToShow2);
                }

                // Publish Image
                publishPicture(mImgToShow1, imgDetectionPub, "bgr8");

                // publish to ros
                publishPicture(mImgToShow2, imgThresholdPub, "mono8");
            }
            //------------- end Ellipse detecting
            mGetNewImage_ = false;
        }

        /// cv::waitKey(int) is needed to draw imshow(...)
        waitKey(33);
    }


}


// checks if specified ellipse can be a recognition candidate
// [in] box - RotatedRect, that fits detected ellipse
// [in] frameHeight - height of source image
// [in] frameWidth - width of source image
// [retval] true if not checks passed
bool DetectingMark::checkEllipseSize(const RotatedRect box, int frameHeight, int frameWidth) {

    bool ellCorrSize_insideImg = false;
    // it may have different angles, but horizontal lenght should be more than vertical
    bool corrEllipseOrientation = false;

    // divisors for dimensions
    float divisorWidth = 8;
    float divisorHeight = 2 * divisorWidth;

    // check in different rotations - different dimensions
    if( (box.angle > 360. - 15.) || (box.angle < 15.) ) {
        // check height vs. width
        if(box.size.height < box.size.width)
            corrEllipseOrientation = true;

        // check if fits inside frame
        if( (box.size.width > frameWidth/divisorWidth) &&
            (box.size.height > frameHeight/divisorHeight) &&
            (box.center.x - box.size.width/2 > 0) &&
            (box.center.y - box.size.height/2 > 0) &&
            (box.center.x + box.size.width/2 < frameWidth) &&
            (box.center.y + box.size.height/2 < frameHeight)
        )
            ellCorrSize_insideImg = true;
    }
    else if( (box.angle > 90. - 15.) && (box.angle < 90. + 15.) ) {
        // check height vs. width
        if(box.size.height > box.size.width)
            corrEllipseOrientation = true;

        // check if fits inside frame
        if( (box.size.height > frameWidth/divisorWidth) &&
            (box.size.width > frameHeight/divisorHeight) &&
            (box.center.x - box.size.height/2 > 0) &&
            (box.center.y - box.size.width/2 > 0) &&
            (box.center.x + box.size.height/2 < frameWidth) &&
            (box.center.y + box.size.width/2 < frameHeight)
        )
            ellCorrSize_insideImg = true;
    }
    else if( (box.angle > 180. - 15.) && (box.angle < 180. + 15.) ) {
        // check height vs. width
        if(box.size.height < box.size.width)
            corrEllipseOrientation = true;

        // check if fits inside frame
        if( (box.size.width > frameWidth/divisorWidth) &&
            (box.size.height > frameHeight/divisorHeight) &&
            (box.center.x - box.size.width/2 > 0) &&
            (box.center.y - box.size.height/2 > 0) &&
            (box.center.x + box.size.width/2 < frameWidth) &&
            (box.center.y + box.size.height/2 < frameHeight)
        )
            ellCorrSize_insideImg = true;
    }
    else if( (box.angle > 270. - 15.) && (box.angle < 270. + 15.) ) {
        // check height vs. width
        if(box.size.height > box.size.width)
            corrEllipseOrientation = true;

        // check if fits inside frame
        if( (box.size.height > frameWidth/divisorWidth) &&
            (box.size.width > frameHeight/divisorHeight) &&
            (box.center.x - box.size.height/2 > 0) &&
            (box.center.y - box.size.width/2 > 0) &&
            (box.center.x + box.size.height/2 < frameWidth) &&
            (box.center.y + box.size.width/2 < frameHeight)
        )
            ellCorrSize_insideImg = true;
    }

    // combine checks result
    if(corrEllipseOrientation && ellCorrSize_insideImg)
        return true;
    else
        return false;
}


// Determines if white pixels count within given shape on given binarized
// image exceeds 80% of total pixels count
// [in] ellipse - dimensions of found ellipse
// [in] threshold_output - image to check ellipse on
// [retval] - true if check passed
bool DetectingMark::checkWhiteInEllipse(const RotatedRect ellipse, Mat threshold_output) {
    // total pixel count
    unsigned int count = 0;
    // white pixel count
    unsigned int countWhite = 0;

    // find region dimensions
    int h_min = ( int )( ellipse.center.y - ellipse.size.width/2 ) + 1;
    int h_max = ( int )( ellipse.center.y + ellipse.size.width/2 ) - 1;

    // loop inside given region vertically
    for ( int i = h_min; i < h_max ; ++i ) {
        // get Y coord.
        int y = abs(i - ellipse.center.y);

        // find region dimensions
        int x_min = ( int )( ellipse.center.x - ( ellipse.size.height/2 )*sqrt( 1 - y*y/((ellipse.size.width/2)*(ellipse.size.width/2)))) + 1;
        int x_max = ( int )( ellipse.center.x + ( ellipse.size.height/2 )*sqrt( 1 - y*y/((ellipse.size.width/2)*(ellipse.size.width/2)))) - 1;

        // loop inside given region horizontaly
        for( int j = x_min; j < x_max; ++j) {
            // count total
            count++;

            // count white
            if( threshold_output.at<uchar>(i,j) )
                countWhite ++;
        }
    }

    /// Check count of white pixels inside wllipse
    if( countWhite > ((int)(0.8*count)) )
        return true;
    else
        return false;
}

// Callback handler for ROS messages passed to control workflow
// standard ROS signature
void DetectingMark::callbackStopPause(const ::std_msgs::Int32& msg) {
    // on different messages set corresponding internal flags
    switch (msg.data) {
        case -2:  //stop Alg
            mWait_ = false;
            mStop_ = true;
            write_log("STOP Message received", LOG_LEVEL_INFO);
            break;
        case -1://pause Alg
            mWait_ = true;
            mStop_ = false;
            write_log("PAUSE Message received", LOG_LEVEL_INFO);
            break;
        case 0://work Alg
            mWait_ = false;
            mStop_ = false;
            write_log("CONTINUE message received", LOG_LEVEL_INFO);
            break;
    }
}


// Callback handler for ROS messages passed to set HSV filter ranges
// standard ROS signature
void DetectingMark::callbackSetHsvRange(const croc_detecting_mark_3::HSVrange& msg) {
    // form debug string with params passed
    stringstream logMsg;
    logMsg << "h[";
    logMsg << msg.Hmax << "-" << msg.Hmin;
    logMsg << "] s[";
    logMsg << msg.Smax << "-" << msg.Smin;
    logMsg << "] v[";
    logMsg << msg.Vmax << "-" << msg.Vmin;
    logMsg << "]";

    write_log(logMsg.str(), LOG_LEVEL_INFO);

    // set passed values to corresponding internal vars.
    // if some var not passed => that internal var not changed
    if(0 < msg.Hmax)
        mHmax_ = msg.Hmax;
    if(0 < msg.Hmin)
        mHmin_ = msg.Hmin;

    if(0 < msg.Smax)
        mSmax_ = msg.Smax;
    if(0 < msg.Smin)
        mSmin_ = msg.Smin;

    if(0 < msg.Vmax)
        mVmax_ = msg.Vmax;
    if(0 < msg.Vmin)
        mVmin_ = msg.Vmin;

    return;
}


// draws a coloured ellipse on image with coords inside ROI
// [in] frame - image to draw on
// [in] ellipseToDraw - RotatedRect containing ellipse
// [in] color - color to draw ellipse
// [in] addROI - flag if RotatedRect is relative to global ROI
void DetectingMark::drawEllipse(cv::Mat frame, RotatedRect ellipseToDraw, cv::Scalar color, bool addROI) {
    // correct coords regarding ROI
    if (mUseROI_==true && addROI==true) {
        ellipseToDraw.center.x+=mROIx_;
        ellipseToDraw.center.y+=mROIy_;
    }

    // draw ellipse
    ellipse( frame, ellipseToDraw, color, 2, 8 );
}


// Checks (further) detected ellipse (white with correct size) if it is a
// target candidate (now - find crossing lines)
// [in] ellCoord - ellipse center coords
// [in] Ellipse - ellipse shape to check
void DetectingMark::cropEllipse_Check(::geometry_msgs::Pose2D ellCoord, RotatedRect Ellipse) {
    // image size storage
    Size imgSize;
    // storage for resized image
    Mat resizeImg;

    // width must be more then height, reverse if neccessary
    if(Ellipse.size.width >= Ellipse.size.height) {
        imgSize.width = Ellipse.size.width;
        imgSize.height = Ellipse.size.height;
    } else {
        imgSize.width = Ellipse.size.height;
        imgSize.height = Ellipse.size.width;
    }

    // Get from the source frame (unmodified) image area underlying given ellipse
    Mat temp(mInputFrame_, cv::Rect((ellCoord.x - imgSize.width/2),(ellCoord.y - imgSize.height/2),
                                    imgSize.width,imgSize.height));

    // Convert resulting image to fixed size 200x200
    Size toResizeImg;
    toResizeImg.width = 200;
    toResizeImg.height = 200;
    cv::resize(temp, resizeImg,toResizeImg);

    // get shape center coords
    Point center;
    center.x = Ellipse.size.width/2;
    center.y = Ellipse.size.height/2;

    // output debug image if setup
    if(mShowVideo_)
        imshow("roi ellipse", resizeImg);

    // If set up - publish found image into topic
    if(mSendPreTarget_)
        publishPicture( resizeImg, imgTargetEllipsePub, "bgr8");

    // Perform further checks if this is a target
    findStraightLines(resizeImg);

    // pause for debugging
    if(mLogLevel_>= LOG_LEVEL_DEBUG)
        // wait 5 sec in ms
        waitKey(5*1000);
}

// modify image with Canny detector and pass it to further processing
// [in] img - image to process
void DetectingMark::findStraightLines(const Mat& img) {
    // copy image for processing
    Mat src = img.clone();
    // processed image
    Mat dst;

    // perform Canny detection
    Canny(src, dst, mCannyThreshold_, mCannyMaxThreshold_, 3);

    // show debug image if setup
    if(mShowVideo_)
        imshow("canny", dst);

    // process image further
    Probabilistic_Hough(dst);
}

// find lines inside image 
// [in] src - source image
void DetectingMark::Probabilistic_Hough( Mat src) {
    // detected lines storage
    vector<Vec4i> p_lines;
    // temporary images
    Mat threshold_output = Mat(src.size(),CV_8UC1);
    Mat probabilistic_hough;
    // randomizer
    RNG rng;

    // convert src image to gray & threshold it
    threshold( src, threshold_output, 150, 255, THRESH_BINARY );
    cvtColor( threshold_output, probabilistic_hough, COLOR_GRAY2BGR );

    /// 2. Use Probabilistic Hough Transform
    HoughLinesP( threshold_output, p_lines, 1, CV_PI/180, 50, 30, 20 );

    /// Show the result
    for( size_t i = 0; i < p_lines.size(); i++ ) {
        Vec4i l = p_lines[i];
        line( probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]), randomColor(rng), 3);
    }

    // Draw  central lines
    line( probabilistic_hough, Point(probabilistic_hough.size().width/2, 0),
          Point(probabilistic_hough.size().width/2, probabilistic_hough.size().height), Scalar(0,0,255), 3);

    line( probabilistic_hough, Point(0, probabilistic_hough.size().height/2),
          Point(probabilistic_hough.size().width, probabilistic_hough.size().height/2), Scalar(0,0,255), 3);


    // show debuf image if setup
    if(mShowVideo_)
        imshow( "probabilistic_name", probabilistic_hough );

    // Publish Image
    publishPicture(probabilistic_hough, imgIntersectionPub, "bgr8");

    // further process image
    findLinesIntersections(p_lines, probabilistic_hough);
}


// detect robust lines intersections from given lines array
// [in] lines - detected lines from pref. image
// [in] srcToDraw - image to show detected intersections
void DetectingMark::findLinesIntersections(vector<Vec4i> lines, Mat srcToDraw) {
    // lines & intersections coords.
    Point2f start1, end1, start2, end2, intersectionPnt;
    // found intersections array
    vector<Vec2i> intersectionPoints;

    // process each given line
    for(int i = 0; i < lines.size(); ++i) {
        start1.x = lines.at(i)[0]; //x start
        start1.y = lines.at(i)[1]; //y start
        end1.x = lines.at(i)[2]; //x end
        end1.y = lines.at(i)[3]; //y end

        // for checking angle difference between lines
        float angle1 = atan((end1.y -  start1.y)/(end1.x -  start1.x)) * 180/M_PI; // angle in degrees

        // loop other (remaining) lines
        for(int j = i+1; j < lines.size(); ++j) {
            start2.x = lines.at(j)[0]; //x start
            start2.y = lines.at(j)[1]; //y start
            end2.x = lines.at(j)[2]; //x end
            end2.y = lines.at(j)[3]; //y end

            // Check angle difference between line
            float angle2 = atan((end2.y -  start2.y)/(end2.x -  start2.x)) * 180/M_PI; // angle in degrees
            if(abs(angle1 - angle2) > mAngleDiff_) { // angle difference between lines should be more than
                if(intersection(start1, end1, start2, end2, intersectionPnt)) {
                    // storage for result output
                    Vec2i inter;
                    // store detected intersection coords
                    inter[0] = (int) intersectionPnt.x;
                    inter[1] = (int) intersectionPnt.y;

                    // check if sound intersect is near middle
                    if((inter[0] > srcToDraw.size().width/4 ) && (inter[0] < srcToDraw.size().width - srcToDraw.size().width/4) &&
                       (inter[1] > srcToDraw.size().height/4) && (inter[1] < srcToDraw.size().height - srcToDraw.size().height/4)) {
                        // draw found intersect
                        ellipse(srcToDraw, RotatedRect(Point(inter[0], inter[1]), Size2f(5,5), 0), Scalar(122,122,0),5);
                        // store that intersect
                        intersectionPoints.push_back(inter);
                    }
                }
            }
        }
    }

    // if found more than 3 intersected lines
    if(intersectionPoints.size() >= 3)
        mHaveIntersection_ = true;

    /// Median filter
    if( medianFilterForPoints(intersectionPoints) && mHaveIntersection_) {
        // show found point
        ellipse(srcToDraw, RotatedRect(Point(intersectionPoints.at(0)[0], intersectionPoints.at(0)[1]), Size2f(5,5), 0), Scalar(0,0,255),5);

        // Calc coordinates on image from source
        if(mEllipseToRoi_.size.width > mEllipseToRoi_.size.height) {
            toPublishPrecisePoint.x = mEllipseToRoi_.center.x +
            (intersectionPoints.at(0)[0] - srcToDraw.size().width/2) * mEllipseToRoi_.size.width/srcToDraw.size().width;
            toPublishPrecisePoint.y = mEllipseToRoi_.center.y +
            (intersectionPoints.at(0)[1] - srcToDraw.size().height/2) * mEllipseToRoi_.size.height/srcToDraw.size().height;
        } else {
            toPublishPrecisePoint.x = mEllipseToRoi_.center.x +
            (intersectionPoints.at(0)[0] - srcToDraw.size().width/2) * mEllipseToRoi_.size.height/srcToDraw.size().width;
            toPublishPrecisePoint.y = mEllipseToRoi_.center.y +
            (intersectionPoints.at(0)[1] - srcToDraw.size().height/2) * mEllipseToRoi_.size.width /srcToDraw.size().height;
        }

        /// Calc coordinate if we use ROI
        if(mUseROI_) {
            toPublishPrecisePoint.x += mROIx_;
            toPublishPrecisePoint.y +=  mROIy_;
        }

        // set flag - new info found
        mNewPublishInfo = true;
    }

    // show debug window if setup
    if(mShowVideo_)
        imshow("intersection", srcToDraw);
}


// return true if only one point left after filtering other by median
// [in, out] points - vector of possible intersection points
// [retval] flag if point found
bool DetectingMark::medianFilterForPoints(vector<Vec2i> points) {
    // center point of other intersections
    double x_center = 0;
    double y_center = 0;

    //if we have no points return false
    if(points.empty() == true)
        return false;

    // there must be only one point left
    while(points.size() > 1) {
        // Calc mass center
        for(int i = 0; i < points.size(); ++i) {
            x_center += points.at(i)[0];
            y_center += points.at(i)[1];
        }

        // Avg previous sum
        x_center = x_center/points.size();
        y_center = y_center/points.size();

        // Find distance from points to mass center
        vector<float> distToCenter;
        float dist = 0;
        for(int i = 0; i < points.size(); ++i) {
            dist = sqrt((points.at(i)[0] - x_center)*(points.at(i)[0] - x_center) +
                        (points.at(i)[1] - y_center)*(points.at(i)[1] - y_center));
            distToCenter.push_back(dist);
        }

        // Find max distance
        float maxDist = 0;
        int maxDistCounter = 0;
        for(int i = 0; i < points.size(); ++i) {
            if(distToCenter.at(i) >= maxDist) {
                maxDist = distToCenter.at(i);
                maxDistCounter = i;
            }
        }
        // Delete point with max distance to mass center
        points.erase(points.begin() + maxDistCounter);
    }

    // if we got until here - it's fine
    return true;
}


// Finds the intersection of two lines, or returns false.
// The lines are defined by (o1, p1) and (o2, p2).
// [in] o1 - first edge of first line
// [in] p1 - second edge of first line
// [in] o2 - first edge of second line
// [in] p2 - second edge of second line
// [out] r - intersection point
// [retval] -  flag of successful search
bool DetectingMark::intersection(
        Point2f o1,
        Point2f p1,
        Point2f o2,
        Point2f p2,
        Point2f &r
) {
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

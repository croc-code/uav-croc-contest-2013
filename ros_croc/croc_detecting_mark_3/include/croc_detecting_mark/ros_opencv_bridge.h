#ifndef ROS_OPENCV_BRIDGE
#define ROS_OPENCV_BRIDGE

/// This class is neede to listen to topic that send images.

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cvaux.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  cv::Mat image;

  bool newImage;

public:
  ImageConverter(  ros::NodeHandle &nh_, std::string topicName)
    : it_(nh_)
  {

    image_sub_ = it_.subscribe(topicName , 1, &ImageConverter::imageCb, this);
    std::cout << "Initialize ImageConverter, subscribe to " << topicName << std::endl;
    newImage = false;    
  }

  ~ImageConverter()
  {
      it_.~ImageTransport();

  }

  cv::Mat getImage()
  {
      newImage = false;
      return image.clone();
  }

  bool isImageUpdate()
  {
      return newImage;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    image = cv_ptr->image.clone();

    newImage = true;

  }
};

#endif // ROS_OPENCV_BRIDGE

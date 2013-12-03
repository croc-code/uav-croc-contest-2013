/*********************************************************	

(c) CROC Inc 2013

This source code demonstrates gazebosim 1.9 plugins we used
for testing our solution for CROC Flying robot competition 
2013 (www.robots.croc.ru ).

gazebo_camera_plugin - plugin that takes images from gazebo
virtual camera sensor and publish it to ros topic.

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/
#include "gazebo.hh"
#include "plugins/CameraPlugin.hh"
#include <transport/transport.hh>
#include <gazebo.hh>
#include <physics/physics.hh>
#include "plugins/CameraPlugin.hh"
#include <common/common.hh>
#include <stdio.h>

#include <ros/ros.h>
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>

#include "image_transport/image_transport.h"

namespace gazebo
   {
     class CameraDump : public CameraPlugin
     {
       public: CameraDump() : CameraPlugin() {}

       public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
       {

         // Load the camera plugin
         CameraPlugin::Load(_parent,_sdf);

         // Init ROS
         std::string name = "ros_camera_plugin_node";
         int argc = 0;
         ros::init(argc, NULL, name);
         this->node = new ros::NodeHandle("~");
         this->itnode = new image_transport::ImageTransport(*this->node);
         this->image_pub = this->itnode->advertise("croc_image", 1, false);
       }

       // Camera new frame event handler
       public: void OnNewFrame(const unsigned char *_image,
           unsigned int _width, unsigned int _height, unsigned int _depth,
           const std::string &_format)
       {
			 // prpare image message
             this->image_msg.header.stamp = ros::Time::now();
             sensor_msgs::fillImage(this->image_msg,
                     sensor_msgs::image_encodings::RGB8,
                     _height,
                     _width,
                     3*_width,
                     (void*)_image);

             // publish to ros topic via ros image transport
             this->image_pub.publish(this->image_msg);

       }

       private: ros::NodeHandle* node;					// ROS node
       private: image_transport::Publisher image_pub;	// ROS image publisher
       private: image_transport::ImageTransport* itnode;	// ROS Image Transport to publish image messages
       private: sensor_msgs::Image image_msg;			// ROS Image messages

     };

     // Register this plugin with the simulator
     GZ_REGISTER_SENSOR_PLUGIN(CameraDump)
   }

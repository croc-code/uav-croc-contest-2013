/*********************************************************

(c) CROC Inc 2013

This source code demonstrates gazebosim 1.9 plugins we used
for testing our solution for CROC Flying robot competition
2013 (www.robots.croc.ru ).

gazebo_lidar_plugin - plugin that publishes laser scan info
from gazebo virtual ray sensor into /Scan topic (substitute
for ros node 'hokuyo_node').

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

#include "math/Pose.hh"
#include "math.h"

#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/RaySensor.hh"

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "sensor_msgs/LaserScan.h"


namespace gazebo
{
  class ROSLidarPlugin : public ModelPlugin
  {

    public: ROSLidarPlugin()
    {

      // init ROS
      std::string name = "ros_lidar_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }


    public: ~ROSLidarPlugin()
    {
      delete this->node;
    }




    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

        // ROS Nodehandle
        this->node = new ros::NodeHandle("~");

        // Init ROS Lidar publisher
        this->pub_lidar = this->node->advertise<sensor_msgs::LaserScan>("/scan",100);

        // Init ray sansor
        sensors::SensorPtr sensor = sensors::SensorManager::Instance()->GetSensor("laser");
        this->raysensor = boost::shared_dynamic_cast<sensors::RaySensor>(sensor);
        this->raysensor->Init();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                   boost::bind(&ROSLidarPlugin::OnUpdate, this));

    }


    // Called by the world update start event
    public: void OnUpdate()
    {
          std::vector<double> rangesgz;
                  current_time = ros::Time::now();

                  // Will scan in 20hz, same as real hokuyo with skip=1 parameter
                  if ((ros::Time::now()-lastLaserPub).toSec()>0.05)
                  {
                      this->raysensor->SetActive(false);

                      lastLaserPub = ros::Time::now();
					  
                      this->lock_.lock();

					  //Create Scan message header
                      sensor_msgs::LaserScan scan2ros;
                      scan2ros.header.stamp=current_time; 
                      scan2ros.header.frame_id="laser";

					  //Create Scan message angle values
                      math::Angle maxAngle = this->raysensor->GetAngleMax();
                      math::Angle minAngle = this->raysensor->GetAngleMin();
                      scan2ros.angle_min=minAngle.Radian();
                      scan2ros.angle_max=maxAngle.Radian();

					  //Create Scan message angle increment value
                      scan2ros.angle_increment=this->raysensor->GetAngleResolution();
					  
					  //Create Scan message time_increment and scan time values. They are not needed, so 
					  //put them to zero
                      scan2ros.time_increment=0.0;
                      scan2ros.scan_time=0.0;
				  
					  //Create Scan message min\max range values
                      double maxRange = this->raysensor->GetRangeMax();
                      double minRange = this->raysensor->GetRangeMin();
                      scan2ros.range_min=minRange;
                      scan2ros.range_max=maxRange;

					  // clear previous ranges and intensities arrays
					  // and resize arrays
                      scan2ros.ranges.clear();
                      scan2ros.intensities.clear();
                      int rayCount = this->raysensor->GetRayCount();
                      int rangeCount = this->raysensor->GetRangeCount();
                      scan2ros.ranges.resize(rayCount);
                      scan2ros.intensities.resize(rayCount);
						
					  // geta laser data from sensor
                      this->raysensor->GetRanges(rangesgz);

					  // fill Scan message range array
                      for (int iray=0;iray<rayCount;iray++)
                      {
                            double rg = this->raysensor->GetRange(iray);
                            double intensity = this->raysensor->GetRetro(iray);
							// add some noise to range and intensity values
                            scan2ros.ranges[iray]=std::min(rg+minRange+GaussianKernel(0,0.01), maxRange);
                            scan2ros.intensities[iray]=std::max(101.0, intensity+GaussianKernel(0,0.01));
                      }

                      this->raysensor->SetActive(true);

					  // publish values
                      this->pub_lidar.publish(scan2ros);
                      this->lock_.unlock();
                  }

                  // *******************************************************
                  ros::spinOnce();

     }



      // Utility for adding noise
      double GaussianKernel(double mu,double sigma)
      {
        double U = (double)rand()/(double)RAND_MAX;
        double V = (double)rand()/(double)RAND_MAX;
        double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
        X = sigma * X + mu;
        return X;
      }


	// times    
	ros::Time current_time, lastLaserPub;


    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

	// virtual laser sensor
    private: sensors::RaySensorPtr raysensor;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock_;

	// ros /Scan publisher
    ros::Publisher pub_lidar;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSLidarPlugin)
}

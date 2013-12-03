/*********************************************************

(c) CROC Inc 2013

This source code demonstrates gazebosim 1.9 plugins we used
for testing our solution for CROC Flying robot competition
2013 (www.robots.croc.ru ).

gazebo_px4flow_plugin - plugin that publishes model altitude
and velocity data into px4flow OpticalFlow topic (substitute
for ros node 'px4flow_node'). It publishes only opt_flow data,
without images.

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

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "include/OpticalFlow.h"


namespace gazebo
{   
  class ROSPX4FlowPlugin : public ModelPlugin
  {

    public: ROSPX4FlowPlugin()
    {

      // init ROS
      std::string name = "ros_px4flow_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }


    public: ~ROSPX4FlowPlugin()
    {
      delete this->node;
    }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

          // ROS Nodehandle
        this->node = new ros::NodeHandle("~");

        // Init ROS PX4Flow publisher
        this->pub_PX4Flow = this->node->advertise<px_comm::OpticalFlow>("/px4flow/opt_flow",1000);

        last_value_changed_time = ros::Time::now();
        last_time = ros::Time::now();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                   boost::bind(&ROSPX4FlowPlugin::OnUpdate, this));

    }


    // Called by the world update start event
    public: void OnUpdate()
    {

          // get current pose of model
          math::Pose current_pose = this->model->GetRelativePose();
          current_time = ros::Time::now();

          // will work in rate ~ 100Hz
          if ((current_time-last_time).toSec()>0.01)
          {
              opt_msg.header.stamp = ros::Time::now();
              // publish ground distance
              // topic published in frequency of 100Hz, but sonar data is changed
              // in frequency of 10Hz only
              if ((current_time-last_value_changed_time).toSec()>0.1)
              {
                  opt_msg.ground_distance = current_pose.pos.z + GaussianKernel(0,0.01) + HangOut(100, current_pose.pos.z);
                  if (opt_msg.ground_distance<0.30) opt_msg.ground_distance = 0.30;
                  last_value_changed_time = current_time;
              }

              // publish velocities
              math::Vector3 vel = this->model->GetRelativeLinearVel();
              opt_msg.velocity_x = vel.x;
              opt_msg.velocity_y = vel.y;

              pub_PX4Flow.publish(opt_msg);

              last_time=current_time;
          }

     }


      // Utility for adding noise
      double GaussianKernel(double mu,double sigma)
      {
        double U = (double)rand()/(double)RAND_MAX;
        double V = (double)rand()/(double)RAND_MAX;
        double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
       // scale to our mu and sigma
        X = sigma * X + mu;
        return X;
      }


      // hangout emulation
      // emulate mirrored ultrasonic signal
      // int tick - how often it should happen (for example 100 ticks in 10hz sonar - once in nearly 10 seconds. Bat it's random too!)
      // double CurrentHeight - current altitude of copter
      double HangOut(int tick, double currentHeight)
      {
        int v = rand() % tick+1;
        int m = rand() % 2+1; // values of 1 and 2;
        if (v==tick)
        {
            ROS_INFO("Hangout! %f", currentHeight*m);
            return currentHeight*m+0.15*m;
        }
        else
            return 0;
      }


    // times
    ros::Time current_time, last_time, last_value_changed_time;

    // Pointer to the model
    private: physics::ModelPtr model;

    private: px_comm::OpticalFlow opt_msg;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    /// \brief A mutex to lock access to fields that are used in message callbacks
    private: boost::mutex lock_;

    // optic flow publisher
    ros::Publisher pub_PX4Flow;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSPX4FlowPlugin)
}

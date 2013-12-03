/*********************************************************

(c) CROC Inc 2013

This source code demonstrates gazebosim 1.9 plugins we used
for testing our solution for CROC Flying robot competition
2013 (www.robots.croc.ru ).

gazebo_flyer_plugin - flight controller realization for
gazebo robot model. It emulates Mikrokopter FK and
cyphy_serial_driver node: it gets target pitch, roll and yaw
and try to change model pose to this PRY values with four
forces which emulate propellers.

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/


#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <stdio.h>

#include <ros/ros.h>
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "math/Vector3.hh"
#include "math/Pose.hh"

#include "sensor_msgs/LaserScan.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"

#include "nav_msgs/Path.h"

#include "geometry_msgs/Twist.h"

#include <control_toolbox/pid.h>

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "include/mikoCmd.h"
#include "include/mikoImu.h"

#include "include/wind.h"
#include "include/windParameters.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"


namespace gazebo
{   
  class ROSFlyerPlugin : public ModelPlugin
  {


      double currentX;
      double currentY;
      double currentTheta;

      double goalX;
      double goalY;
      double goalTheta;

      control_toolbox::Pid pid_velocity_x;
      control_toolbox::Pid pid_velocity_y;
      control_toolbox::Pid pid_velocity_t;

      ros::Time current_time, last_time;

      ros::Time lastLaserPub;

      croc::Wind wind;



    public: ROSFlyerPlugin()
    {

      // Init ROS
      std::string name = "ros_flyer_plugin_node";
      int argc = 0;
      ros::init(argc, NULL, name);
    }


    public: ~ROSFlyerPlugin()
    {
      delete this->node;
    }




    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

          // ROS Nodehandle
        this->node = new ros::NodeHandle("~");

        // Init ROS Subscribers and publishers
        this->subCmd = this->node->subscribe("/c_command", 100, &ROSFlyerPlugin::cmdCallback, this );
        this->subWinPrm = this->node->subscribe("/WindParameters", 100, &ROSFlyerPlugin::windprmCallback, this );
        this->pubImu = this->node->advertise<cyphy_serial_driver::mikoImu>("/mikoImu",100);
        this->pubWind = this->node->advertise<geometry_msgs::WrenchStamped>("/Wind",100);

        // init wind
        wind.initWind(math::Vector3(1, 0, 0), 5, 15, 1, 2 , 0.1, 0.2, 1, 2);

        //init pids
        pitch_pid.initPid(120, 0, 40, 100, -100);
        roll_pid.initPid(120, 0, 40, 100, -100);
        yaw_pid.initPid(4, 0.4, 2, 0.2, -0.2);


        ros::WallDuration(3).sleep();

        // hover throttle force init
        hover_throttle = 5.2;

        // previous time init
        last_time = ros::Time::now();

        // max cmd values init
        max_pitch_roll_cmd=3000.0;
        max_yaw_cmd=30.0;

        // wait 10ms
        ros::WallDuration(0.01).sleep();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                   boost::bind(&ROSFlyerPlugin::OnExternalUpdate, this));



    }


    // Called by the world update start event
    public: void OnExternalUpdate()
    {

        // get current model pose and get current pitch, roll and yaw
        math::Pose current_pose = this->model->GetRelativePose();
        current_yaw_deg = rad_to_deg(current_pose.rot.GetYaw());
        double current_pitch = rad_to_deg(current_pose.rot.GetPitch());
        double current_roll = rad_to_deg(current_pose.rot.GetRoll())*(-1.0);

        current_time = ros::Time::now();

        // emulate controller to work with frequency of 300Hz
        if ((current_time-last_time).toSec()>0.003)
        {

            ros::Duration delta_t = current_time-last_time;
            last_time = current_time;

            // get pitch/roll/yaw commands as values of pids
            pitch_cmd = pitch_pid.updatePid(current_pitch - pitch_goal, delta_t);
            roll_cmd = roll_pid.updatePid(current_roll - roll_goal, delta_t);
            yaw_cmd = yaw_pid.updatePid(current_yaw_deg - yaw_goal, delta_t);

            if(pitch_cmd>max_pitch_roll_cmd) pitch_cmd = max_pitch_roll_cmd;
            if(pitch_cmd<-max_pitch_roll_cmd) pitch_cmd = -max_pitch_roll_cmd;
            if(roll_cmd>max_pitch_roll_cmd) roll_cmd = max_pitch_roll_cmd;
            if(roll_cmd<-max_pitch_roll_cmd) roll_cmd = -max_pitch_roll_cmd;
            if(yaw_cmd>max_yaw_cmd) yaw_cmd = max_yaw_cmd;
            if(yaw_cmd<-max_yaw_cmd) yaw_cmd = -max_yaw_cmd;

        }

        // pitch and roll command values are relative forces wich added (or substitite) to main throttle on propelleres platform (pretty fair)
        this->model->GetChildLink("croc_flyer_v1::north_platform")->AddRelativeForce(math::Vector3(0.0, 0.0, throttle+pitch_cmd));
        this->model->GetChildLink("croc_flyer_v1::south_platform")->AddRelativeForce(math::Vector3(0.0, 0.0, throttle-pitch_cmd));
        this->model->GetChildLink("croc_flyer_v1::east_platform")->AddRelativeForce(math::Vector3(0.0, 0.0, throttle-roll_cmd));
        this->model->GetChildLink("croc_flyer_v1::west_platform")->AddRelativeForce(math::Vector3(0.0, 0.0, throttle+roll_cmd));
        // yaw cmd is simply relative angular velocity (not so fair, but ok)
        this->model->GetChildLink("croc_flyer_v1::center")-> SetAngularVel(math::Vector3(0.0, 0.0, -yaw_cmd/500));


        // Add WIND
        // Wind - is relative force, periodic in strength and direction
        // This force applies to center of robot
        math::Vector3 currentWind = wind.getCurrentWind();
        this->model->GetChildLink("croc_flyer_v1::center")->AddRelativeForce(currentWind);

        //publish wind wrench for displaying this info in croc_gs
        geometry_msgs::WrenchStamped windWrench;
        windWrench.wrench.force.x = currentWind.x;
        windWrench.wrench.force.y = currentWind.y;
        windWrench.wrench.force.z = currentWind.z;
        windWrench.header.frame_id="/world";
        windWrench.header.stamp = ros::Time::now();
        pubWind.publish(windWrench);

        // publish imu data (real model data, without noise and errors)
        cyphy_serial_driver::mikoImu imuMsg;
        imuMsg.anglePitch = current_pose.rot.GetPitch();
        imuMsg.angleRoll = current_pose.rot.GetRoll();
        pubImu.publish(imuMsg);

    }


    // command callback
    private: void cmdCallback (const cyphy_serial_driver::mikoCmdConstPtr msg)
    {
        // get new main throttle & pitch\roll\yaw goals
        throttle = msg->throttle;
        pitch_goal = msg->pitch/10.0;
        roll_goal = msg->roll/10.0;
        yaw_goal = current_yaw_deg+msg->yaw;
    }

    // wind parameters callcbak (from croc_gs)
    private: void windprmCallback (const croc_gs::windParametersConstPtr msg)
    {
        // get parameters
        math::Vector3 base;
        base.x = msg->baseVector.x;
        base.y = msg->baseVector.y;

        // reinit wind
        wind.initWind(base,
                      msg->minAlpha,  msg->maxAlpha,
                      msg->minBeta, msg->maxBeta,
                      msg->minGamma, msg->maxGamma,
                      msg->minKappa, msg->maxKappa);
    }


    // radians to degree transformation
    private: double rad_to_deg(double rad)
    {
          return rad*57.2957795;
    }


    // Pointer to the model
    private: physics::ModelPtr model;


    private: double max_pitch_roll_cmd; // max pitch and roll command (force)
    private: double max_yaw_cmd;        // max yaw command (force)


    private: double throttle;           // current main throttle
    private: double pitch_goal;         // curent pithc goal
    private: double roll_goal;          // current roll goal
    private: double yaw_goal;           // current yaw goal relative to current yaw position
    private: double current_yaw_deg;    // current yaw position

    double pitch_cmd;                   // pitch command (force)
    double roll_cmd;                    // roll command (force)
    double yaw_cmd;                     // yaw command (force)

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // ROS Nodehandle
    private: ros::NodeHandle* node;

    private: double hover_throttle; // hover throttle

    //PIDs
    private: control_toolbox::Pid pitch_pid;
    private: control_toolbox::Pid roll_pid;
    private: control_toolbox::Pid yaw_pid;

    // ROS Subscribers and Publishers
    ros::Subscriber subCmd;
    ros::Subscriber subWinPrm;
    ros::Publisher pubImu;
    ros::Publisher pubWind;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSFlyerPlugin)
}

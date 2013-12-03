/*
 *  Mikrokopter Flight control Serial Interface
 *  Copyright (C) 2010, CYPHY lab
 *  Inkyu Sa <i.sa@qut.edu.au>
 *
 *  http://wiki.qut.edu.au/display/cyphy
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MIKO_FLIGHTCONTROL_FLIGHTCONTROL_H
#define MIKO_FLIGHTCONTROL_FLIGHTCONTROL_H

#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <stdarg.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>


#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <sys/time.h>
#include "serialinterface.h"
#include "sensor_msgs/Imu.h"  //For height
#include "cyphy_serial_driver/mikoImu.h"
#include "cyphy_serial_driver/mikoCmd.h"
#include <control_toolbox/pid.h> //For PID height control


#define TRUE 1
#define FALSE 0
#define YES 1
#define NO 0

#define FC_ADDRESS 2

// MAX when hovering
//#define MAX_PITCH 10
//#define MAX_ROLL 10
//#define MAX_YAW 100

#define YAW_SCALE_FACTOR 10.
#define ACC_X_K 35
#define ACC_Y_K 35
#define BATT_MAX 200


#define ANGLE_PITCH 0
#define ANGLE_ROLL 1

#define ACC_X 2
#define ACC_Y 3
#define GYRO_YAW 4

#define HEIGHT 5

#define ACC_Z_ADC 6

#define GAS 7 // #define GAS 9 //

#define COMPASS 8

#define BATT 9 // #define BATT 8

#define RECIEVER 10

#define ANGLE_YAW 11 // GYRO_COMPASS

#define MOTOR_FRONT 12
#define MOTOR_REAR 13
#define MOTOR_LEFT 14
#define MOTOR_RIGHT 15

#define ACC_Z_FILTER 20 // not true
#define STICK_YAW 23 // not true
#define STICK_PITCH 24 // not true
#define STICK_ROLL 25 // not true

#define GPS_Nick 30
#define GPS_Roll 31



#define HIGH_VALUE 5 // HohenWert air pressure sensor value


#define PI 3.141592653589793238462643383279
#define DEG_TO_RAD(x) (x*0.0174532925f)
#define RAD_TO_DEG(x) (x*57.2957795f)
#define BOUNDARY 50   // in mm
#define g 9.81   //9.8 m/s^2
#define K 0.5   // Complementary filter gain
#define Kp_Pitch 12.
#define Kp_Roll 12.

#define Kp_vel_X 3.
#define Kp_vel_Y 12.
#define Kp_pos_X 10.
#define Kp_pos_Y 10.

#define GAS_PARAM 3.32

typedef struct
{
  double x;
  double y;
  double z;
  double theta;
} __attribute__((packed)) Position_t;

typedef struct
{
  int pitch;
  int roll;
  int yaw;
} Control_t;

typedef struct
{
  double x;
  double y;
  double z;
} Velocity_t;


struct str_Data3D
{
   signed int  angle[3]; // pitch, roll, yaw in 0,1°
   signed char Centroid[3];
   signed char reserve[5];
};

typedef struct
{
	uint8_t Digital[2];
	int16_t Analog[32];    // Debugvalues
        //unsigned char Digital[2];
        //signed int Analog[32];
} __attribute__((packed)) DebugOut_t;


namespace miko
{
  class FlightControl
  {
    private:

      ros::Timer timer_;
    
      double freq_;
      std::string port_;
      int speed_;
      bool Throttle_Direction;

// Although I tried to change data type of Pitch and Roll, Yaw as int16_t, it only generated enormous delay
// of request data(Angle pitch, AccX...). I have no idea what is the problem. Therefore, roll back to the
// original data structure of Mikrokopter. 
#if 1        
  typedef struct
  {
	uint8_t	Digital[2];
	uint8_t	RemoteButtons;
	int8_t	Pitch;
	int8_t	Roll;
	int8_t	Yaw;
	uint8_t  Throttle;
	int8_t	Height;
	uint8_t	free;
	uint8_t	Frame;
	uint8_t	Config;
    } ExternControl_t;
#endif


typedef struct
{
  double AnglePitch;
  double AngleRoll;
  double AngleYaw;
  double ACCX;
  double ACCY;
  double ACCZ;
} __attribute__((packed)) Attitude_t;




typedef struct
{
  double x;
  double y;	 
  double z;	 
  double yaw;	
}__attribute__((packed)) DesiredPosition_t;

    ros::Publisher pub;         //mikoImu pub
    ros::Publisher pub_stdImu;  //sensors_msgs::Imu pub

    sensor_msgs::Imu stdImuMsg;
    tf::Quaternion tfImuQuaternion;
    geometry_msgs::Quaternion geometryImuQuaternion;

    //ros::Publisher pub_pose2D;

    
    uint64_t time;
    
    DesiredPosition_t DesiredPosition;
    Attitude_t MyAttitude;

    public:
      SerialInterface* serialInterface_;

      FILE *fd,*fd_h,*fd_debug;
      cyphy_serial_driver::mikoImu mikoImu;
      ros::Time last_time;
      ros::Time last_time_;
      ros::Time current_time;
      ros::Time current_time_;

      ExternControl_t	ExternControl;
      ros::Subscriber mikoCmdSubscriber;
      int myYawAngle;

      FlightControl ();
      virtual ~FlightControl();
      void AddCRC(uint16_t datelen);
      void SendOutData(uint8_t cmd, uint8_t addr, uint8_t numofbuffers, ...);
      void enablePolling (uint16_t request, uint16_t interval);
      void spin (const ros::TimerEvent & e);
      void mikoCmdCallback (const cyphy_serial_driver::mikoCmd& msg);
      
      unsigned long long time_helper(void);
  }; // end class FlightControl
} //end namespace miko

#endif

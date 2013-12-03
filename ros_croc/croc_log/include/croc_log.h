#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <croc_command/PID.h>
#include <croc_Pose3D/Pose3D.h>
#include <cyphy_serial_driver/mikoImu.h>
#include <cyphy_serial_driver/mikoCmd.h>
#include "sensor_msgs/Range.h"
#include "px_comm/OpticalFlow.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"


// subscribers
ros::Subscriber pose3d_sub;
ros::Subscriber mikoimu_sub;
ros::Subscriber mikocmd_sub;
ros::Subscriber pid_height_sub;
ros::Subscriber range_height_sub;
ros::Subscriber px4flow_sub;
ros::Subscriber pid_x_sub;
ros::Subscriber pid_x_second_sub;
ros::Subscriber global_goal_sub;
ros::Subscriber local_goal_sub;

// file
FILE *logFile;
std::string filename;

// logged messages 
croc_Pose3D::Pose3D pose3DMsg;
cyphy_serial_driver::mikoImu mikoImuMsg;
cyphy_serial_driver::mikoCmd mikoCmdMsg;
croc_command::PID PIDHeightMsg;
sensor_msgs::Range rangeMsg;
px_comm::OpticalFlow optMsg;
croc_command::PID PIDXMsg;
croc_command::PID PIDXSecondMsg;
geometry_msgs::PoseStamped globalGoalMsg;
geometry_msgs::Pose2D localGoalMsg;


//calculated horizontal speed
double x_dif_vel;
double y_dif_vel;
ros::Time last_dif_time;
croc_Pose3D::Pose3D last_pose;

// speed integration variables
double x_int;
double y_int;
ros::Time last_int_time;


// callbacks
void poseCallback (const croc_Pose3D::Pose3DConstPtr& msg);
void mikoImuCallback (const cyphy_serial_driver::mikoImuConstPtr& msg);
void mikoCmdCallback (const cyphy_serial_driver::mikoCmdConstPtr& msg);
void PIDHeightCallback (const croc_command::PIDConstPtr& msg);
void rangeHeightCallback (const sensor_msgs::RangeConstPtr& msg);
void px4flowCallback (const px_comm::OpticalFlowConstPtr& msg);
void PIDXCallback (const croc_command::PIDConstPtr& msg);
void PIDXSecondCallback (const croc_command::PIDConstPtr& msg);
void GlobalGoalCallback (const geometry_msgs::PoseStampedConstPtr& msg);
void LocalGoalCallback (const geometry_msgs::Pose2DConstPtr& msg);


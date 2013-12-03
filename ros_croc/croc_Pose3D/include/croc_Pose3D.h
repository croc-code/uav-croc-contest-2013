#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "geometry_msgs/Pose2D.h"
#include "croc_Pose3D/Pose3D.h"
#include "OpticalFlow.h"
#include "cyphy_serial_driver/mikoImu.h" 
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"


ros::Publisher pub;                 // main Pose3D publisher
ros::Publisher odom_pub;            // odometry publisher (for navigation stack)

ros::Subscriber pose2D_sub;         // pose2D laser scan matcher subscriber
ros::Subscriber range_height_sub;   // range ultrasonic subscriber
ros::Subscriber imu_sub;            // mikrokopter inner imu subscriber
ros::Subscriber px4flow_sub;        // px4flow optical flow subscriber


// variables for z-speed calculation, based
// on ultrasonic range data
double current_v_z;         // current z-speed
float prev_z;               // previous altitude, obtained from sonar
float prev_prev_z;          // last altitude, obtained from sonar, which was different from peceding value
ros::Time prev_z_time;      // time of prev_z
ros::Time prev_prev_z_time; // time of prev_prev_z
bool isVZFirstRun;          // mark for first run of z-velocity calculation
double prev_measured;       // previous raw measured data
// ---------------------------------------

//Node params
int hangout_handling_mode;  // mode of sonar hangout handlig. 0 - without handling, 1 - dividing by 2 or 3 handling, 2 - previous measurement taking;
double hangout_handling_min_altitude; // minimum altitude when hangout algotithms work;

//---- Calibration rutine ---
ros::Time start_calibration;
float v_x_error;
float v_y_error;
ros::Duration delay;
double calibrationDelay;
ros::Time last_read;
bool is_calibrating;

//-- Filter params
double velocity_lpf_factor;

ros::Time of_last_time;          // time for flow position integration

croc_Pose3D::Pose3D msg;            // croc_Pose3D current node published message


void calculateLinearZVelocity(ros::Time stamp);     // calculate z-velocity based on sonar data

// callbacks
void rangeHeightCallback (const sensor_msgs::RangeConstPtr& rangeMsg);

void imuHeightCallback (const cyphy_serial_driver::mikoImuConstPtr& imuMsg);

void pose2DCallback (const geometry_msgs::Pose2DConstPtr& pose2DMsg);

void opticalFlowCallback (const px_comm::OpticalFlowConstPtr optMsg);


#include "math.h"

#include "ros/ros.h"
#include "croc_command/mikoCmd.h"
#include "croc_command/PID.h"
#include "croc_command/State.h"
#include "croc_Pose3D/Pose3D.h"
#include "geometry_msgs/Pose2D.h"

#include <control_toolbox/pid.h>

#define RAD_TO_DEG(x) (x*57.2957795f)

#define RATE 0.02
#define LANDING_SLOW_PERIOD 10.0

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

enum {P_GAIN, I_GAIN, D_GAIN, I_MAX, I_MIN};
enum {GROUND, STARTING_ENGINES, TAKEOFF, AIR, LANDING, EXTREME_LANDING};

typedef struct
{
	double x;
	double y;
	double z;
	double theta;
    double pitch;
    double roll;
} Position_t;

ros::Publisher cmd_pub;         // publisher of FC commands
ros::Publisher pid_height_pub;  // publisher of altitude pid values for logging
ros::Publisher pid_x_pub;       // publisher of x-pid values for logging
ros::Publisher pid_x_second_pub;// publisher of second x-pid values for logging
ros::Publisher pid_y_pub;       // publisher of y-pid values for logging

ros::Publisher state_pub;       // publisher of state

ros::Subscriber pose3d_sub; // subscriber of current Pose3D
ros::Subscriber state_sub;  // subscriber of robot state landing, taking off, etc)
ros::Subscriber goal_sub;    // subscriber of robot goal position (Pose2D)


// params
bool isEnabled;             // is overall control enabled
bool is2DPoseEnabled;       // is 2DPose control enabled
bool is2DPoseByVelocity;    // true - stabilizing by velocity, false - by destinations
bool isOpticFlowVelocity; // true - incorporate velicity estimation by optic flow
bool isHoverTest;          // true if testing hover, goal speed is always 0
double startingEnginePeriod;// period in seconds for starting engine.
double height_gain[5];      // gains for height pid
double theta_gain[5];       // gains for theta pid
double xy_gain[5];          // gains for 2dposition pid
double nav_xy_gain[5];      // gains for navigation pid, controls roll/pitch angles
double wp_radius;           // Way point radis in m.
double xy_vel_goal_gain[5]; // gains for horizontal velocities goal pid
double maxthrottle;         // max throttle we send to FC
double minthrottle;         // min throttle we send to FC
double hoverthrottle;       // value, that is added to height pid
double landSpeedMin;        // minimal speed of landing in cm\sec
double landSpeedMax;        // max speed of landing in cm\sec
double landHeightForSpeedMinOn; // altitude, when minimal landing speed is turning on
double sonarDeadZone;            // sonar dead zone (used during landing)
double takeoffSpeed;        // sped of takeoff in cm\sec
double flightAltitude;      // Altitude of horizontal flight
double landHeight;          // height, when we assume that robot is on the ground
double yawScale;            // scale const for theta pid
double max_pitch;           // max absolute pitch angle we send to FC
double max_roll;            // max absolute roll angle we send to FC
double max_yaw;             // max absolute yaw angle we send to FC
double max_horizontal_velocity; // max horizontal velocity in meters per second
double velocity_error_scale;    // we set goal x_velocity as max(x_error/velocity_error_scale, max_horizontal_velocity). Same for y.
double min_flow_height;     // Minimal heigh to starting estimation using optic flow
double max_lidar_height;    // Maximum height sill lidar estimation is used.
bool d_part_as_pitch_roll_derivative;    // Take d_part of x-y pids as pitch\roll derivatives.



// control message (for mk flight controller)
croc_command::mikoCmd controlMsg;

// PID msgs for logging
croc_command::PID PIDHeightMsg;
croc_command::PID PIDXMsg;
croc_command::PID PIDYMsg;


Position_t goalPosition;    	// goal position as we get in from goal topic
Position_t currentPosition; 	// current position as we get it from Pose3D topic
Position_t oldPosition;     	// position on previous step
Position_t u;
double goalVelocity_x;        	// goal velocity in meters per second
double goalVelocity_y;        	// goal velocity in meters per second

double estimatedZVel;			// estimated z-velocity

int currentState;				// current state of robot
int previousState;				// previous state of robot
double landingAltitudeDelta;  	// change of goal height while landing (calc on landingspeed, time interval, etc)
double takeoffAltitudeDelta;  	// change of goal height while takeoff (calc on tekoffspeed, time interval, etc)
double throttle;               	// throttle for engine starting sequence
bool isNearGroundWhileLanding; 	// flag which become true when we pass altitude landHeightForSpeedMinOn while landing sequence


int landingSignalCount;        	// count of signals that robot is on the ground.
int deadZoneLandingSignalCount; // count of signals that robot is in sonar dead zone.
int inTheAirSignalCount;        // count of signals that robot is in the air (near hovering position)


// PIDs
control_toolbox::Pid pid_z;
control_toolbox::Pid pid_x;
control_toolbox::Pid pid_y;
control_toolbox::Pid pid_theta;
control_toolbox::Pid pid_velocity_goal_x;
control_toolbox::Pid pid_velocity_goal_y;

// times
ros::Time current_time;
ros::Time last_time;
ros::Time start_landing_time;
ros::Time last_time_post2d_stabilized;


// Callback for subscriber of current Pose3D
void poseCallback (const croc_Pose3D::Pose3DConstPtr& msg);

// Callback for subscriber of robot state landing, taking off, etc)
void stateCallback (const croc_command::StateConstPtr& msg);

// Callback for subscriber of robot goal position (pose2D)
void goalCallback (const geometry_msgs::Pose2DConstPtr& msg);


// logic of altitude stabilizing
void altitudeStabilize();


// logic of moving to goal pose2D point
void pose2DStabilize();

// moving to goal realisation with velocity PIDs
void pose2DStabilize_Velocity();

// state switcher
// in:
//  newstate - new state to switch in
//  oldstate - state, we switch from
void changeState(int newstate, int oldstate);


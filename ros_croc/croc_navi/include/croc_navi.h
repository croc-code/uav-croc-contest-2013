#include "ros/ros.h"
#include "croc_Pose3D/Pose3D.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GridCells.h"
#include "croc_command/State.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Empty.h"

// have two kind of states. Control state determines low level states of robot.
// Navigation state determined high level of states and used for global goal setting.
enum {GROUND, STARTING_ENGINES, TAKEOFF, AIR, LANDING};   // control states
enum {STARTING, FORWARD, FIND_LANDING_MARK, RETURNING}; // navigation states
enum {NORTH, SOUTH, EAST, WEST};

ros::Publisher localGoalPub;        // local goal for croc_command
ros::Publisher globalGoalPub;       // global goal for planner
ros::Publisher statePub;            // state publisher for robot
ros::Publisher detectingStatePub;   // state of detecting algorithm (on/off/pause)

ros::Subscriber poseSub;            // Pose3D Subscriber
ros::Subscriber scanSub;            // Scan Subscriber
ros::Subscriber pathSub;            // Global Path Subscriber
ros::Subscriber stateSub;           // Robot State Subscriber
ros::Subscriber goalSub;            // Global Goal Subscriber
ros::Subscriber costmapSub;         // costmap Subscriber
ros::Subscriber markerSub;          // marker position Subscriber

ros::ServiceClient clearMapsSrvClient;

nav_msgs::Path globalPath;          // global path
geometry_msgs::PoseStamped globalGoal; // global goal
croc_Pose3D::Pose3D currentPose;    // current Pose of robot
croc_command::State currentControlState;   // current control State of robot
nav_msgs::GridCells obstacles;      // current costmap (obstacles)
sensor_msgs::LaserScan scan;        // current lidar scan
geometry_msgs::Pose2D markerPose;   // current marker position
geometry_msgs::Pose2D localGoal;    // local goal message to publish

double startingDistanceToNorth=0; // Distance to north and east walls at starting moment.
double startingDistanceToEast=0;  // Needed when return back home.

double goalX, goalY;  // goal variables

double newLocalGoalDistance; // distance to new local goal (calculated dynamicly)

// params
bool navigationOn;		// turn on navigation
bool autoHeading;		// turn on autoheading of quad
bool returnBasedOnWalls;	// correct returning goal with distance to walls
double newMaxLocalGoalDistance; // maximum distance local goal
double globalGoalReachDistance;   // distance to global goal when global goal is marked as reached
double doNotTurnDistance;	// distance to local goal when autoheading turns off
double doNotMoveTheta;	// min theta error when quad stops moving and only turns to minimize theta error. only works if auto_heading is on.
double rangeLength;		// range length in meters
double rangeWidth;		// rnage width in meters

int currentNavigationState; // current Navigation state (STARTING, FORWARD, FIND_LANDING_MARK, RETURNING)
int currentReturningStep;   // substates of robot while it's returns

bool markWasFound;			// flag: true - landing mark was once found.
bool wasClearMapInTheAir;   // flag: Costmap was once cleared, when quad was in the air


// callbacks
void pathCallback (const nav_msgs::PathConstPtr msg);
void poseCallback (const croc_Pose3D::Pose3DConstPtr msg);
void stateCallback (const croc_command::StateConstPtr msg);
void goalCallback (const geometry_msgs::PoseStampedConstPtr msg);
void costmapCallBack(const nav_msgs::GridCellsConstPtr msg);
void scanCallback(const sensor_msgs::LaserScanConstPtr msg);
void markerCallback(const geometry_msgs::Pose2DConstPtr msg);


void navigationLoop();							// main navigation loop


void updateLocalGoal();             			// set goal for croc_command node
void updateGlobalGoal(double x, double y);		// set goal for global planner
bool isGlobalGoalReached(double reachDistance);	// check if quad near (< reachDistance) global goal

double getDistanceTo(double x, double y);  		// get distance from current position to (x;y)
void switchDetectingAlgo(int state);    		// switch state of detecting algorithm

double getNearestWallDistance(int wallId);      // get nearest wall distance. wallId is taken from enum {NORTH, SOUTH, EAST, WEST};





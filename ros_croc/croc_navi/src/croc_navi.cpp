/*********************************************************	

(c) CROC Inc 2013

This source code demonstrates navigation, take-off, landing
and visual mark detection algorithms  we’ve developed for
CROC Flying robot competition 2013 (www.robots.croc.ru ).

croc_navi - analog of local planner. We do not use local
planners from ros-nav-stack, which calculate velocities
based on global path and local_cost_map. Instead of this,
we subscribe "path", "scan", "Pose3D" and "DetectingMark"
topics and depending on current STATE of robot, publish local
goal point to topic "Goal" for croc_command node (which move
robot to it) and global goal for global_planner (which
generates new global path and etc.).

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/

#include "croc_navi.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "croc_navi");
    ros::NodeHandle n;

    // set states
    currentControlState.state = GROUND;
    currentNavigationState = STARTING;
    currentReturningStep = 0;
    globalGoal.pose.position.x = 0;
    globalGoal.pose.position.y = 0;

    // set goals
    goalX=0;
    goalY=0;

	// set flags
    markWasFound = false;
    wasClearMapInTheAir = false;


    //publishers
    localGoalPub = n.advertise<geometry_msgs::Pose2D>("Goal", 100);
    globalGoalPub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    statePub = n.advertise<croc_command::State>("StateCmd", 100);
    detectingStatePub = n.advertise<std_msgs::Int32>("LandingFieldDetectionAlgorithmStartStop", 100);

    //subscribers
    pathSub = n.subscribe("/move_base/NavfnROS/plan", 100, pathCallback);
    poseSub = n.subscribe("/Pose3D", 100, poseCallback);
    stateSub = n.subscribe("/StateCurrent", 100, stateCallback);
    goalSub = n.subscribe("/move_base_simple/goal", 100, goalCallback);
    costmapSub = n.subscribe("/move_base/global_costmap/obstacles", 100, costmapCallBack);
    scanSub = n.subscribe("/scan", 100, scanCallback);
    markerSub = n.subscribe("/LandingFieldPos", 100, markerCallback);

    //services
    clearMapsSrvClient = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

    // params
    if (!n.getParam("/croc_navi/navigation_on", navigationOn)) navigationOn=false;

    if (!n.getParam("/croc_navi/new_local_goal_distance", newMaxLocalGoalDistance)) newMaxLocalGoalDistance=0.6;
    if (!n.getParam("/croc_navi/auto_heading", autoHeading)) autoHeading=false;
    if (!n.getParam("/croc_navi/return_based_on_walls", returnBasedOnWalls)) returnBasedOnWalls=true;

    if (!n.getParam("/croc_navi/global_goal_reach_distance", globalGoalReachDistance)) globalGoalReachDistance=0.9;
    if (!n.getParam("/croc_navi/do_not_turn_distance", doNotTurnDistance)) doNotTurnDistance=0.3;
    if (!n.getParam("/croc_navi/do_not_move_theta", doNotMoveTheta)) doNotMoveTheta=1.57;

    if (!n.getParam("/croc_navi/range_length", rangeLength)) rangeLength=40;
    if (!n.getParam("/croc_navi/range_width", rangeWidth)) rangeWidth=10;





    while (ros::ok())
        {

           if (navigationOn==true) navigationLoop();

            ros::WallDuration(0.02).sleep();
            ros::spinOnce();

        }

            return 0;
}



void navigationLoop()
{
  
    // if we are in STARTING navi-state, standing on the GROUND
    // run engines and takeoff.
    if (currentNavigationState==STARTING && currentControlState.state==GROUND)
    {

        ROS_INFO_ONCE("Navi: STARTING");

        // pause detecting algorithm
        switchDetectingAlgo(-1);

        // transition between STARTING_ENGINES->TAKEOFF->AIR
        // takes place in croc_commnand node, because it's a
        // fully technical question

        // Start engines immidiatly if goal is passed
        if (globalGoal.pose.position.x!=0.0 || globalGoal.pose.position.y!=0.0 )
        {
          currentControlState.state=STARTING_ENGINES;
          statePub.publish(currentControlState);
        }

    }


    if (currentNavigationState==STARTING && currentControlState.state==AIR)
    {
        // if robot in the air (reached given altitude) - let's move
        currentNavigationState=FORWARD;

        // cooldown for 2 seconds
        ros::WallDuration(2).sleep();

        ROS_INFO_ONCE("Navi: GOING FORWARD");
    }


    // if we are in the air and have FORWARD navi-state, set global goal.
    // The landing mark is somewhere there
    if (currentNavigationState==FORWARD && currentControlState.state==AIR)
    {

        // going to symmetrical point in the second part of range
        updateGlobalGoal(0, startingDistanceToEast + rangeWidth/4);

       	// turn on detecting algo
        switchDetectingAlgo(0);
       
		// while flying, check if we find landing mark in right position
		if (markerPose.x<10 && markerPose.x>-startingDistanceToNorth && markerPose.y>startingDistanceToEast+0.5 && markerPose.y<startingDistanceToEast + (rangeWidth/2) - 0.5 )
        {
            ROS_INFO_ONCE("NAVI SEARCHING: moving to landing mark");
            updateGlobalGoal(markerPose.x, markerPose.y);
            if (isGlobalGoalReached(2)) currentNavigationState=FIND_LANDING_MARK;
            markWasFound = true;
        }

        // if we didnt find landing mark, but reach global goal - return home. mission failed. 
        if (markWasFound == false && isGlobalGoalReached(0.6)) currentNavigationState = RETURNING;

		// updating local goal;
        updateLocalGoal();


        ROS_INFO_ONCE("Navi: FORWARD IN THE AIR");
    }

   
    // if we find landing mark and robot is in the air - robot should go there and land
    if (currentNavigationState==FIND_LANDING_MARK && currentControlState.state==AIR)
    {
        // landing mark coordinates must be already set as our global goal position.
        // so - move there and check if we are reach our goal
        ROS_INFO_ONCE("Navi: FIND LANDING MARK");
        autoHeading = false;

        // pause detecting algorithm
        switchDetectingAlgo(-1);

        updateLocalGoal();

        if (isGlobalGoalReached(globalGoalReachDistance))
        {
            // start landing sequence
            currentControlState.state=LANDING;
            statePub.publish(currentControlState);
        }
    }

    // if we find landing mark and robot is on the ground - robot should fly again in the air
    if (currentNavigationState==FIND_LANDING_MARK && currentControlState.state==GROUND)
    {

        currentNavigationState = RETURNING;

        // clear map
        std_srvs::Empty srv;
        if (clearMapsSrvClient.call(srv))  ROS_INFO("Map clearing service ground call Ok"); else ROS_INFO("Map clearing service ground call failure!");


        // change local goal to current position
        localGoal.x = currentPose.x;
        localGoal.y = currentPose.y;
        localGoalPub.publish(localGoal);

        // cooldown for 2 seconds
        ros::WallDuration(2).sleep();

		// start engines
        currentControlState.state=STARTING_ENGINES;
        statePub.publish(currentControlState);

        ROS_INFO_ONCE("Navi: LAND ON LANDING MARK");


    }

    // if we are coming home and in the air - set goal to starting point (x=0, y=0)/ landing mark
    // should be there
    if (currentNavigationState==RETURNING && currentControlState.state==AIR)
    {

        ROS_INFO_ONCE("Navi: RETURNING BACK");

        // clear map once again
        if (wasClearMapInTheAir == false)
        {
            std_srvs::Empty srv;
            if (clearMapsSrvClient.call(srv))  ROS_INFO("Map clearing service air call Ok"); else ROS_INFO("Map clearing service air call failure!");
            wasClearMapInTheAir = true;
        }



        switch(currentReturningStep)
        {
            case 0:
                ROS_INFO_ONCE("SIMPLE RETURNING: moving to 0,0");
                updateGlobalGoal(0, 0);
                if (isGlobalGoalReached(5) && currentPose.y<startingDistanceToEast) currentReturningStep = 1;
                break;
            case 1:
                if (returnBasedOnWalls)
                {
                    ROS_INFO_ONCE("Wall Based Correction");

                    // set gloabal goal based on walls
                    double goalY = currentPose.y+getNearestWallDistance(EAST)-startingDistanceToEast;
                    double goalX = currentPose.x-getNearestWallDistance(NORTH)+startingDistanceToNorth;

                    if (goalX<-900) goalX = currentPose.x - 2;
                    updateGlobalGoal(goalX, goalY);

                 }

        }
        
        updateLocalGoal();


        // check for goal achivement
        if (isGlobalGoalReached(globalGoalReachDistance))
        {

             ROS_INFO("Navi: RETURNING BACK - LANDING");
            // start landing sequence
            currentControlState.state=LANDING;
            statePub.publish(currentControlState);
        }

    }

}



//  get new local goal from global path to move directly
void updateLocalGoal()
{
    // iterate throuh global path values;
    // find next point in path, which is {newLocalGoalDistance} far from
    // current position and set it as new local goal.
    // And publish it

    // current position: currentPose
    geometry_msgs::PoseStamped pose;
    double distance=0;

    // updateing newLocalGoalDistance value;
    newLocalGoalDistance=std::min(newMaxLocalGoalDistance, getDistanceTo(globalGoal.pose.position.x, globalGoal.pose.position.y));

    if (globalPath.poses.size()>0)
    {
        // search for msot distant node in {newLocalGoalDistance} range from
        // current position
        double selectedIndex=0;
        for (int i=0; i<(int)globalPath.poses.size()-1; i++)
        {
            // select next node
            pose = globalPath.poses[i];
            distance = getDistanceTo(pose.pose.position.x, pose.pose.position.y);
            if (distance < newLocalGoalDistance)
               selectedIndex=i;
        }
        pose = globalPath.poses[selectedIndex];

        // goal theta = direction to local goal.
        double goalTheta = std::atan2((float)(pose.pose.position.y - currentPose.y), (float)(pose.pose.position.x - currentPose.x));
        double errorTheta = currentPose.yaw - goalTheta;
        if (errorTheta>M_PI) errorTheta = errorTheta - 2*M_PI;
        if (errorTheta<-M_PI) errorTheta = errorTheta + 2*M_PI;

        // if autoHeading is on, robot will turn towards goal
        if (autoHeading)
        {
            // check, if current distance is small - do not turn
            if (distance>doNotTurnDistance)
            {
                localGoal.theta = goalTheta;
            }

        }

        // if errorTheta is very big - do not move. But only if disstance to goal is not that small when
        // robot is forbidden to turn (we do not want to stuck in such situation)
        // Also if autoheading is off - we will fly anyway.
        if (std::abs(errorTheta)<doNotMoveTheta || distance<=doNotTurnDistance || autoHeading==false)
        {

            // permission to move is granted
            localGoal.x = pose.pose.position.x;
            localGoal.y = pose.pose.position.y;
        }
        else
        {
            // stop futher action and turn until errrorTheta is small enough!
        }
        localGoalPub.publish(localGoal);
    }

}


// update global goal by publishing it in the moving_base topic
// x, y - goal coordinates
void updateGlobalGoal(double x, double y)
{
    globalGoal.header.frame_id="world";
    globalGoal.header.stamp=ros::Time::now();
    globalGoal.pose.position.x=x;
    globalGoal.pose.position.y=y;
    globalGoal.pose.position.z=0;
    globalGoalPub.publish(globalGoal);
}



// switch state of detecting algo
// 0 - run
// -1 - pause
// -2 - stop
void switchDetectingAlgo(int state)
{
    std_msgs::Int32 detectingStateMsg;  // state of detecting algo
    detectingStateMsg.data = state;
    detectingStatePub.publish(detectingStateMsg);

}



/* -------- measuring Utils -------- */


// get 2D-distance from UAV current position to specified point
// x, y - point coordinates
double getDistanceTo(double x, double y)
{
      return sqrt(pow(currentPose.x-x,2) + pow(currentPose.y-y,2));
}


// check, if UAV reaches global goal
bool isGlobalGoalReached(double reachDistance)
{
    // current position: currentPose
    // global goal: globalGoal
    double distance = getDistanceTo(globalGoal.pose.position.x, globalGoal.pose.position.y);

    if (distance < reachDistance) return true;

    return false;
}


// get distance from UAV to specified wall
// based on obstacles map
// if no wall is found - return big result (1000 meters)
double getNearestWallDistance(int wallId)
{

    double result=1000; // if we will not find wall - return big result

    for (int i=0; i<(int)obstacles.cells.size(); i++)
    {
      geometry_msgs::Point cell = obstacles.cells[i];


      switch (wallId)
      {
        case NORTH:

            // search for nearest obstacles with X < Current.X and Y in (Current.Y-0.5,Current.Y+0.5)
            if (cell.x-currentPose.x<0 && currentPose.x-cell.x<result && std::abs(cell.y-currentPose.y)<0.5) result = currentPose.x-cell.x;
            break;
        case SOUTH:

            // search for nearest obstacles with X > Current.X and Y in (Current.Y-0.5,Current.Y+0.5)
            if (cell.x-currentPose.x>0 && cell.x-currentPose.x<result && std::abs(cell.y-currentPose.y)<0.5) result = cell.x-currentPose.x;
            break;
        case WEST:

            // search for nearest obstacles with X in (Current.X-0.5, Current.X+0.5) and Y < Current.Y
            if (cell.y-currentPose.y<0 && currentPose.y-cell.y<result && std::abs(cell.x-currentPose.x)<0.5) result = currentPose.y-cell.y;
            break;
        case EAST:

            // search for nearest obstacles with X in (Current.X-0.5, Current.X+0.5) and Y > Current.Y
            if (cell.y-currentPose.y>0 && cell.y-currentPose.y<result && std::abs(cell.x-currentPose.x)<0.5) result = cell.y - currentPose.y;
            break;
      }
    }
     return result;
}



// get distance to obstacle at given angle
// based on lidar info
double getRayDistanceByAngle(double angle)
{

    // move theta to [0, 2pi]
    double theta = currentPose.yaw;
    if (theta < 0) theta = theta + 2*M_PI;

    double worldMinAngle = theta - 2.26889 + M_PI/2; // pi/2 is for situation when hokuyo is turned on p\2
                                                   // 2.26889 - is hokuyo sector

    double angleResolution = scan.angle_increment;
    if (angleResolution==0.0) return -1;

    int index = (angle - worldMinAngle)/angleResolution;
    if (index<0) index = index + 2*M_PI/angleResolution;

    if (index>(int)(scan.ranges.size())-1) return -1.0;

    return scan.ranges[index];
}



/*======== Callbacks =========*/

void pathCallback (const nav_msgs::PathConstPtr msg)
{

    globalPath = *msg;
}

void poseCallback (const croc_Pose3D::Pose3DConstPtr msg)
{

    currentPose = *msg;
}

void stateCallback (const croc_command::StateConstPtr msg)
{
    currentControlState = *msg;

}

void goalCallback (const geometry_msgs::PoseStampedConstPtr msg)
{
    globalGoal = *msg;

}

void scanCallback(const sensor_msgs::LaserScanConstPtr msg)
{
    scan = *msg;
}

void markerCallback(const geometry_msgs::Pose2DConstPtr msg)
{
    markerPose = *msg;
}


void costmapCallBack(const nav_msgs::GridCellsConstPtr msg)
{
    obstacles = *msg;

    // get starting position of copter
    if (startingDistanceToNorth==0 && startingDistanceToEast==0)
    {
        startingDistanceToNorth = getRayDistanceByAngle(M_PI);
        startingDistanceToEast = getRayDistanceByAngle(M_PI/2);

        ROS_INFO_ONCE ("DISTANCE TO NORTH: %f", startingDistanceToNorth);
        ROS_INFO_ONCE ("DISTANCE TO EAST: %f", startingDistanceToEast);

    }
}


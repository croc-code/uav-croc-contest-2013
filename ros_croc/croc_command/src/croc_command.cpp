/*********************************************************	

(c) CROC Inc 2013

This source code demonstrates navigation, take-off, landing
and visual mark detection algorithms  we’ve developed for
CROC Flying robot competition 2013 (www.robots.croc.ru ).

croc_command - main control node which performes altitude 
and horizontal moving\stabilization tasks. It contains logic
full of PID controllers, landing and take-off detections and
control parameters of different kind. This node gets current
position from Pose3D and local goal point from croc_navi and
tries to move robot to designated goal.

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/

#include "croc_command/croc_command.h"

int main(int argc, char**argv)
{


	// initial state values
    currentState = GROUND;
    previousState = GROUND;


    ros::init(argc, argv, "croc_command");
	ros::NodeHandle n;
	cmd_pub = n.advertise<croc_command::mikoCmd>("c_command", 100);
	pid_height_pub = n.advertise<croc_command::PID>("c_h_pid", 100);
    pid_x_pub = n.advertise<croc_command::PID>("c_x_pid", 100);
    pid_x_second_pub = n.advertise<croc_command::PID>("c_x_second_pid", 100);
    pid_y_pub = n.advertise<croc_command::PID>("c_y_pid", 100);
    state_pub = n.advertise<croc_command::State>("StateCurrent", 100);
    pose3d_sub = n.subscribe("Pose3D", 100, poseCallback);
    state_sub = n.subscribe("StateCmd", 100, stateCallback);
    goal_sub = n.subscribe("Goal", 100, goalCallback);


	// initial landing\takeoff values
    landingSignalCount = 0;
    deadZoneLandingSignalCount=0;
    inTheAirSignalCount=0;
    isNearGroundWhileLanding = false;

	// initial control values
    controlMsg.throttle=0;
	controlMsg.pitch=0;
	controlMsg.yaw=0;
	controlMsg.roll=0;

	// initial position values
    goalPosition.x = 0;
    goalPosition.y = 0;
	goalPosition.theta = 0;
    oldPosition.x = 0;
    oldPosition.y = 0;
    oldPosition.z = 0;
    oldPosition.theta = 0;


    // Height params
    if (!n.getParam("/croc_command/flight_altitude", flightAltitude)) flightAltitude=100;
    if (!n.getParam("/croc_command/land_speed_goal_max", landSpeedMax)) landSpeedMax=9; // cm per sec
    if (!n.getParam("/croc_command/land_speed_goal_min", landSpeedMin)) landSpeedMin=9; // cm per sec
    if (!n.getParam("/croc_command/land_height_for_turn_min_speed_on", landHeightForSpeedMinOn)) landHeightForSpeedMinOn=60; // cm per sec

    if (!n.getParam("/croc_command/takeoff_speed_goal", takeoffSpeed)) takeoffSpeed=20; // cm per sec
    if (!n.getParam("/croc_command/land_height", landHeight)) landHeight=11;
    if (!n.getParam("/croc_command/maxthrottle", maxthrottle)) maxthrottle = 130;
    if (!n.getParam("/croc_command/minthrottle", minthrottle)) minthrottle = 72;
    if (!n.getParam("/croc_command/hover", hoverthrottle)) hoverthrottle = 0;
    if (!n.getParam("/croc_command/sonar_deadzone", sonarDeadZone)) sonarDeadZone = 0;

    // Height stabilization PID
	if (!n.getParam("/croc_command/h_p_gain", height_gain[P_GAIN])) height_gain[P_GAIN] = 2;
	if (!n.getParam("/croc_command/h_i_gain", height_gain[I_GAIN])) height_gain[I_GAIN] = 1;
	if (!n.getParam("/croc_command/h_d_gain", height_gain[D_GAIN])) height_gain[D_GAIN] = 0.3;
	if (!n.getParam("/croc_command/h_i_max", height_gain[I_MAX])) height_gain[I_MAX] = 20;
	if (!n.getParam("/croc_command/h_i_min", height_gain[I_MIN])) height_gain[I_MIN] = -20;


    if (!n.getParam("/croc_command/starting_engine_period", startingEnginePeriod)) startingEnginePeriod=5;

    
    // 2DPose params

    // common
    if (!n.getParam("/croc_command/yaw_scale", yawScale)) yawScale = 10;
    if (!n.getParam("/croc_command/max_pitch", max_pitch)) max_pitch = 5;
    if (!n.getParam("/croc_command/max_roll", max_roll)) max_roll = 5;
    if (!n.getParam("/croc_command/max_yaw", max_yaw)) max_yaw = 1;

    // Theta PID
    if (!n.getParam("/croc_command/t_p_gain", theta_gain[P_GAIN])) theta_gain[P_GAIN] = 1;
    if (!n.getParam("/croc_command/t_i_gain", theta_gain[I_GAIN])) theta_gain[I_GAIN] = 0.1;
    if (!n.getParam("/croc_command/t_d_gain", theta_gain[D_GAIN])) theta_gain[D_GAIN] = 0.0;
    if (!n.getParam("/croc_command/t_i_max", theta_gain[I_MAX])) theta_gain[I_MAX] = 10;
    if (!n.getParam("/croc_command/t_i_min", theta_gain[I_MIN])) theta_gain[I_MIN] = -10;

    // X-Y PID
    if (!n.getParam("/croc_command/xy_p_gain", xy_gain[P_GAIN])) xy_gain[P_GAIN] = 1.0;
    if (!n.getParam("/croc_command/xy_i_gain", xy_gain[I_GAIN])) xy_gain[I_GAIN] = 0.1;
    if (!n.getParam("/croc_command/xy_d_gain", xy_gain[D_GAIN])) xy_gain[D_GAIN] = 0.0;
    if (!n.getParam("/croc_command/xy_i_max", xy_gain[I_MAX])) xy_gain[I_MAX] = 10;
    if (!n.getParam("/croc_command/xy_i_min", xy_gain[I_MIN])) xy_gain[I_MIN] = -10;

    // X-Y Velocities goal PID
    if (!n.getParam("/croc_command/xy_vel_goal_p_gain", xy_vel_goal_gain[P_GAIN])) xy_vel_goal_gain[P_GAIN] = 0.4;
    if (!n.getParam("/croc_command/xy_vel_goal_i_gain", xy_vel_goal_gain[I_GAIN])) xy_vel_goal_gain[I_GAIN] = 0.1;
    if (!n.getParam("/croc_command/xy_vel_goal_d_gain", xy_vel_goal_gain[D_GAIN])) xy_vel_goal_gain[D_GAIN] = 0.7;
    if (!n.getParam("/croc_command/xy_vel_goal_i_max", xy_vel_goal_gain[I_MAX])) xy_vel_goal_gain[I_MAX] = 0.1;
    if (!n.getParam("/croc_command/xy_vel_goal_i_min", xy_vel_goal_gain[I_MIN])) xy_vel_goal_gain[I_MIN] = -0.1;

    if (!n.getParam("/croc_command/max_horizontal_velocity", max_horizontal_velocity)) max_horizontal_velocity = 0.1;


    // common params
    if (!n.getParam("/croc_command/enable", isEnabled)) isEnabled = false;
    if (!n.getParam("/croc_command/enable2DPose", is2DPoseEnabled)) is2DPoseEnabled = false;


	// init PIDs
    pid_x.initPid(xy_gain[P_GAIN], xy_gain[I_GAIN], xy_gain[D_GAIN], xy_gain[I_MAX], xy_gain[I_MIN]);
    pid_y.initPid(xy_gain[P_GAIN], xy_gain[I_GAIN], xy_gain[D_GAIN], xy_gain[I_MAX], xy_gain[I_MIN]);
    pid_z.initPid(height_gain[P_GAIN], height_gain[I_GAIN], height_gain[D_GAIN], height_gain[I_MAX], height_gain[I_MIN]);
    pid_theta.initPid(theta_gain[P_GAIN], theta_gain[I_GAIN], theta_gain[D_GAIN], theta_gain[I_MAX], theta_gain[I_MIN]);
    pid_velocity_goal_x.initPid(xy_vel_goal_gain[P_GAIN], xy_vel_goal_gain[I_GAIN], xy_vel_goal_gain[D_GAIN], xy_vel_goal_gain[I_MAX], xy_vel_goal_gain[I_MIN]);
    pid_velocity_goal_y.initPid(xy_vel_goal_gain[P_GAIN], xy_vel_goal_gain[I_GAIN], xy_vel_goal_gain[D_GAIN], xy_vel_goal_gain[I_MAX], xy_vel_goal_gain[I_MIN]);



	// logging msg
	PIDHeightMsg.p_gain = height_gain[P_GAIN];
	PIDHeightMsg.i_gain = height_gain[I_GAIN];
	PIDHeightMsg.d_gain = height_gain[D_GAIN];
	PIDHeightMsg.i_max = height_gain[I_MAX];
	PIDHeightMsg.i_min = height_gain[I_MIN];
	PIDHeightMsg.goal = goalPosition.z;
	PIDHeightMsg.hover = hoverthrottle;


    last_time = ros::Time::now();
    //wait some time
    ros::WallDuration(RATE).sleep();


    while (ros::ok())
	{

		// height stabilization
		current_time = ros::Time::now();

		// switch for control states
        switch (currentState)
        {
            case GROUND:

                controlMsg.pitch=0;
                controlMsg.yaw=0;
                controlMsg.roll=0;
                controlMsg.throttle = 0;
                goalPosition.z=0;
                throttle = 0;

                //reset pids (to clear i-part, etc...)
                pid_x.reset();
                pid_y.reset();
                pid_z.reset();
                pid_theta.reset();
                pid_velocity_goal_x.reset();
                pid_velocity_goal_y.reset();

            break;

            case STARTING_ENGINES:
                // Gently increase gas until hoverthrottle minus 20 is reached
                // We want it take near startingEnginePeriod

                if (controlMsg.throttle < hoverthrottle - 20)
                {
                    double throttleDelta = (hoverthrottle - 20.0) * (current_time-last_time).toSec() / startingEnginePeriod;
                    throttle = throttle + throttleDelta;
                    controlMsg.throttle = throttle;
                }
                else
                {
                    changeState(TAKEOFF, STARTING_ENGINES);
                }
                break;

            case TAKEOFF:


                // start to change goal altitude with speed equal to given takeoffSpeed
                takeoffAltitudeDelta = takeoffSpeed * (current_time-last_time).toSec();
                goalPosition.z = goalPosition.z + takeoffAltitudeDelta;


                altitudeStabilize();
                if (is2DPoseEnabled) pose2DStabilize();


                // if we reach given altitide - change state to AIR
                if (goalPosition.z>flightAltitude)
                {
                    goalPosition.z=flightAltitude;
                }

				// if we are near flightAltitude - increment in-the-air signal
                if (currentPosition.z>flightAltitude-10 && goalPosition.z == flightAltitude) inTheAirSignalCount++;

				// if number of signals more then 5 - we are trully in the air.
                if (inTheAirSignalCount>5) {inTheAirSignalCount=0; changeState(AIR, TAKEOFF);}
                break;


            case AIR:

				// just flight 

                altitudeStabilize();
                if (is2DPoseEnabled) pose2DStabilize();

                break;


            case LANDING:

				
				// if we are inside sonar dead zone - simple lower throttle 
                if (currentPosition.z < sonarDeadZone || deadZoneLandingSignalCount>4)
                {
                    // will lower throttle every three steps.
                    if (deadZoneLandingSignalCount % 3 == 0) controlMsg.throttle = controlMsg.throttle-1;
                    if (controlMsg.throttle>hoverthrottle) controlMsg.throttle=hoverthrottle;
                    deadZoneLandingSignalCount++;
                    if (controlMsg.throttle < 1)
                    {
                        deadZoneLandingSignalCount = 0;
                        landingSignalCount = 0;
                        isNearGroundWhileLanding = false;
                        changeState(GROUND, LANDING);
                        ROS_INFO("Landed.");
                    }
                }
                else
                {
                    // check landing status
                    // if we have more than 9 values less than landHeight sequentially - we assume that robot was landed
                    if (currentPosition.z < landHeight) landingSignalCount++; else landingSignalCount = 0;
                    if (landingSignalCount > 9)
                    {
                        deadZoneLandingSignalCount = 0;
                        landingSignalCount = 0;
                        isNearGroundWhileLanding = false;
                        changeState(GROUND, LANDING);
                        ROS_INFO("Landed.");

                    }

                    // Begin to change goal altitude with speed value = landSpeedMax (takenfrom params).
                    // But first N seconds (LANDING_SLOW_PERIOD) we will change altitude slowly,
                    // gradually increasing rate of change to minimize error derivative
                    if ((current_time - start_landing_time).toSec()<LANDING_SLOW_PERIOD)
                        landingAltitudeDelta = landSpeedMax * (current_time-last_time).toSec()*( (current_time - start_landing_time).toSec() / LANDING_SLOW_PERIOD );
                    else
                    {
                        // check if we are near ground
                        if (currentPosition.z < landHeightForSpeedMinOn) isNearGroundWhileLanding=true;
                        //near ground -> landSpeedMin, Max speed else.
                        if (isNearGroundWhileLanding)
                            landingAltitudeDelta = landSpeedMin * (current_time-last_time).toSec();
                        else
                            landingAltitudeDelta = landSpeedMax * (current_time-last_time).toSec();
                    }
                    // set new goal altitide
                    goalPosition.z = goalPosition.z - landingAltitudeDelta;


                    altitudeStabilize();
                }

                if (is2DPoseEnabled) pose2DStabilize();

                break;

             case EXTREME_LANDING:
                // will lower throttle every step.
                controlMsg.throttle = controlMsg.throttle-1;

                if (controlMsg.throttle < 1)
                {
                    changeState(GROUND, EXTREME_LANDING);
                    ROS_INFO("Extreme Landed.");
                }

                if (is2DPoseEnabled) pose2DStabilize();

                break;

        }

        last_time = current_time;

        if (isEnabled) cmd_pub.publish(controlMsg);

        ros::WallDuration(RATE).sleep();
		ros::spinOnce();
	}

        return 0;
}


// Pose3D callback
void poseCallback (const croc_Pose3D::Pose3DConstPtr& msg)
{
	currentPosition.z=msg->z;
    currentPosition.x=msg->x;
    currentPosition.y=msg->y;
    currentPosition.theta=msg->yaw;
    currentPosition.pitch = msg->pitch;
    currentPosition.roll = msg->roll;

    estimatedZVel = msg -> calculated_linear_velocity.z;

}


// state callback (from navi node)
void stateCallback (const croc_command::StateConstPtr& msg)
{
    ROS_INFO("state changed from %d to %d", previousState, msg->state);
    if (msg->state != previousState) changeState(msg->state, currentState);
}


// local position goal callback 
void goalCallback (const geometry_msgs::Pose2DConstPtr& msg)
{
    goalPosition.x = msg->x;
    goalPosition.y = msg->y;
    goalPosition.theta = msg->theta;
}



// change states of robot
void changeState(int newstate, int oldstate)
{
    currentState = newstate;
    previousState = oldstate;

    // save time of landing beginning

    switch (newstate)
    {
        case GROUND:
            ROS_INFO("On the ground from %d.", oldstate);
            break;
        case STARTING_ENGINES:
            ROS_INFO("Starting engines from %d.", oldstate);
            break;
        case TAKEOFF:
            ROS_INFO("Taking off from %d.", oldstate);
            break;
        case AIR:
            ROS_INFO("Altitude reached from %d.", oldstate);
            break;
        case LANDING:
            start_landing_time=ros::Time::now();
            ROS_INFO("Landing from %d.", oldstate);
            break;
    }

    // publish new state to topic - navi node need this
    croc_command::State state;
    state.state = newstate;
    state_pub.publish(state);

}


// altitude stabilization
void altitudeStabilize()
{
    
    double p, d, i, dt; //pid errors
    
    dt = (current_time - last_time).toSec();
    double error = currentPosition.z - goalPosition.z; // P-error
    error = constrain(error, -90, 90);
    double d_part = constrain(estimatedZVel, -50, 50); // D-part

    //update PID, sending velocity as D-part
    u.z = pid_z.updatePid(error, d_part, current_time-last_time);

    // get PID-errors for logging
    pid_z.getCurrentPIDErrors(&p, &i, &d);

    // publish pid-errors
    PIDHeightMsg.p_error = p;
    PIDHeightMsg.i_error = i;
    PIDHeightMsg.d_error = d;
    PIDHeightMsg.pid_value = u.z;
    PIDHeightMsg.goal = goalPosition.z;
    pid_height_pub.publish(PIDHeightMsg);

    // add hover throttle (from config) to PID-value
    u.z = u.z + hoverthrottle;

    // check throttle value for min\max
    if (u.z>maxthrottle) u.z = maxthrottle;
    if (u.z<minthrottle) u.z = minthrottle;

    controlMsg.throttle = u.z;

}



// pose 2D stabilization
void pose2DStabilize()
{
    // To 2D stabilization in a rate of 100Hz.
    if ((current_time - last_time_post2d_stabilized).toSec() > 0.009) pose2DStabilize_Velocity();
}



// Stabilization and moving by horizontal velocity obtained from lidar
void pose2DStabilize_Velocity()
{

    ros::Duration delta_t = current_time - last_time_post2d_stabilized;

    if (oldPosition.x == currentPosition.x && oldPosition.y == currentPosition.y) {
        return;
    }

    // transform postition errors to robot body
    double x_error=(currentPosition.x - goalPosition.x)*cos(currentPosition.theta) + (currentPosition.y - goalPosition.y)*sin(currentPosition.theta);
    double y_error=(currentPosition.x - goalPosition.x)*sin(currentPosition.theta)*(-1.0) + (currentPosition.y - goalPosition.y)*cos(currentPosition.theta);

    x_error = constrain(x_error, -5, 5);
    y_error = constrain(y_error, -5, 5);

    // transfer velocity to robot body
    double x_vel_current = ((currentPosition.x - oldPosition.x)*cos(currentPosition.theta) + (currentPosition.y - oldPosition.y)*sin(currentPosition.theta))/delta_t.toSec();
    double y_vel_current = ((currentPosition.x - oldPosition.x)*sin(currentPosition.theta)*(-1.0) + (currentPosition.y - oldPosition.y)*cos(currentPosition.theta))/delta_t.toSec();


    // get goals velocities as result of pid, based on position errors.
    double x_vel_goal = pid_velocity_goal_x.updatePid(x_error, delta_t);  
    double y_vel_goal = pid_velocity_goal_y.updatePid(y_error, delta_t);

    // limit goal velocities
    if (x_vel_goal > max_horizontal_velocity) x_vel_goal = max_horizontal_velocity;
    if (x_vel_goal < -max_horizontal_velocity) x_vel_goal = -max_horizontal_velocity;
    if (y_vel_goal > max_horizontal_velocity) y_vel_goal = max_horizontal_velocity;
    if (y_vel_goal < -max_horizontal_velocity) y_vel_goal = -max_horizontal_velocity;


	// yaw PID
    double t_error=RAD_TO_DEG(currentPosition.theta)-RAD_TO_DEG(goalPosition.theta);
    // minimize z_error - it must be less than 180 degrees
    if (t_error>180) t_error = t_error - 360;
    if (t_error<-180) t_error = t_error + 360;


	// control pitch and roll as result of "velocity" PID
    controlMsg.pitch = pid_x.updatePid(x_vel_current - x_vel_goal, delta_t);
    controlMsg.roll = pid_y.updatePid(y_vel_current - y_vel_goal, delta_t);
    controlMsg.yaw = pid_theta.updatePid(t_error, delta_t) * (-1.0)/ yawScale;


    /* ---- publish pid logs    ---- */
    double p, d, i; //pid errors
    // get PID-errors for logging
    pid_velocity_goal_x.getCurrentPIDErrors(&p, &i, &d);
    // publish pid-errors
    PIDXMsg.p_error = p;
    PIDXMsg.i_error = i;
    PIDXMsg.d_error = d;
    PIDXMsg.pid_value = pid_velocity_goal_x.getCurrentCmd();
    PIDXMsg.goal = x_vel_goal;

    pid_x_pub.publish(PIDXMsg);

    // get PID-errors for logging
    pid_velocity_goal_y.getCurrentPIDErrors(&p, &i, &d);
    // publish pid-errors
    PIDYMsg.p_error = p;
    PIDYMsg.i_error = i;
    PIDYMsg.d_error = d;
    PIDYMsg.pid_value = controlMsg.roll;
    PIDYMsg.goal = y_vel_goal;

    pid_y_pub.publish(PIDYMsg);

    // get seconf PID-errors for logging
    pid_x.getCurrentPIDErrors(&p, &i, &d);
    // publish pid-errors
    PIDXMsg.p_error = p;
    PIDXMsg.i_error = i;
    PIDXMsg.d_error = d;
    PIDXMsg.pid_value =pid_x.getCurrentCmd();
    PIDXMsg.goal = x_vel_current - x_vel_goal;
    pid_x_second_pub.publish(PIDXMsg);

    /* ----  ------    ---- */


	// limit controls
    if (std::abs(controlMsg.yaw)>max_yaw) controlMsg.yaw = max_yaw * copysign(1.0, controlMsg.yaw);
    if (std::abs(controlMsg.pitch)>max_pitch) controlMsg.pitch = max_pitch * copysign(1.0, controlMsg.pitch);
    if (std::abs(controlMsg.roll)>max_roll) controlMsg.roll = max_roll * copysign(1.0, controlMsg.roll);


    last_time_post2d_stabilized = current_time;
    oldPosition = currentPosition;
}


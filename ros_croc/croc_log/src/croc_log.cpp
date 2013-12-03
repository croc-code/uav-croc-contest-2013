/*********************************************************	

(c) CROC Inc 2013

This source code demonstrates navigation, take-off, landing
and visual mark detection algorithms  we’ve developed for
CROC Flying robot competition 2013 (www.robots.croc.ru ).

croc_log - node writes csv-log with data from mostly all
solution topics.

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/


#include "croc_log.h"



int main(int argc, char**argv)
{
	ros::init(argc, argv, "croc_log");
	ros::NodeHandle n;

    last_int_time = ros::Time::now();

	// init variables
	x_dif_vel=0;
	y_dif_vel=0;
	x_int=0;
	y_int=0;

	// init subscribers
    pose3d_sub = n.subscribe("Pose3D", 100, poseCallback);
	mikoimu_sub = n.subscribe("mikoImu", 100, mikoImuCallback);	
	mikocmd_sub = n.subscribe("c_command", 100, mikoCmdCallback);
	pid_height_sub = n.subscribe("c_h_pid", 100, PIDHeightCallback);
    range_height_sub = n.subscribe("ultrasound", 100, rangeHeightCallback);
    px4flow_sub = n.subscribe("/px4flow/opt_flow", 1000, px4flowCallback);
    pid_x_sub = n.subscribe("c_x_pid", 100, PIDXCallback);
    pid_x_second_sub = n.subscribe("c_x_second_pid", 100, PIDXSecondCallback);
    global_goal_sub = n.subscribe("/move_base_simple/goal", 100, GlobalGoalCallback);
    local_goal_sub = n.subscribe("Goal", 100, LocalGoalCallback);

	// init filename
	if (!n.getParam("/croc_log/filename", filename)) filename="croc_log";
	
	filename = filename+"-XXXXXX";
	int fd = mkstemp((char*)filename.c_str());
        logFile = fdopen(fd, "w");

	// csv header
    fprintf(logFile, "TIME; POSE3D_X; POSE3D_Y; POSE3D_Z; POSE3D_PITCH; POSE3D_ROLL; POSE3D_YAW; POSE3D_ACCX; POSE3D_ACCY; POSE3D_ACCZ; POSE3D_CALC_VELX; POSE3D_CALC_VELY; POSE3D_CALC_VELZ; IMU_AnglePitch; IMU_AngleRoll; IMU_AccPitch; IMU_AccRoll; IMU_YawGyro; IMU_Height; IMU_AccZ; IMU_Gas; IMU_Compass; IMU_Voltage; IMU_Reciever; IMU_GyroCompass; IMU_Motor1; IMU_Moror2; IMU_Motor3; IMU_Motor4; IMU_16; IMU_17; IMU_18; IMU_19; IMU_Servo; IMU_Hovergas; IMU_CurrentA; IMU_Capacity; IMU_HeightSetpoint; IMU_25; IMU_26; IMU_CompassSetpoint; IMU_I2CError; IMU_BLLimit; IMU_GPSPitch; IMU_GPSRoll; CMD_PITCH; CMD_ROLL; CMD_YAW; CMD_THROTTLE; H_PID_P_GAIN; H_PID_I_GAIN; H_PID_D_GAIN; H_PID_I_MAX; H_PID_I_MIN; H_PID_GOAL; H_PID_P_ERROR; H_PID_I_ERROR; H_PID_D_ERROR; H_PID_HOVER_CONST; H_PID_VALUE; SUB_RANGE; SUB_RANGE_TIMESTAMP; X_POS_INTEGRATED; Y_POS_INTEGRATED; X_VEL_DIF; Y_VEL_DIF; 1_X_PID_GOAL; 1_X_PID_P_ERROR; 1_X_PID_I_ERROR; 1_X_PID_D_ERROR; 1_X_PID_CMD; 2_X_PID_GOAL; 2_X_PID_P_ERROR; 2_X_PID_I_ERROR; 2_X_PID_D_ERROR; 2_X_PID_CMD; GLOBAL_GOAL_X; GLOBAL_GOAL_Y; LOCAL_GOAL_X; LOCAL_GOAL_Y;\n");


    ros::WallDuration(0.02).sleep();

	while (ros::ok())
	{

        fprintf(logFile, "%f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %d; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f; %f;  \n",
						ros::Time::now().toSec(),						
						pose3DMsg.x, pose3DMsg.y, pose3DMsg.z, 
						pose3DMsg.pitch, pose3DMsg.roll, pose3DMsg.yaw, 
						pose3DMsg.linear_acceleration.x, pose3DMsg.linear_acceleration.y, pose3DMsg.linear_acceleration.z,
                        pose3DMsg.calculated_linear_velocity.x, pose3DMsg.calculated_linear_velocity.y, pose3DMsg.calculated_linear_velocity.z,
                        mikoImuMsg.debugData[0], mikoImuMsg.debugData[1], mikoImuMsg.debugData[2],
						mikoImuMsg.debugData[3], mikoImuMsg.debugData[4], mikoImuMsg.debugData[5], 
						mikoImuMsg.debugData[6], mikoImuMsg.debugData[7], mikoImuMsg.debugData[8], 
						mikoImuMsg.debugData[9], mikoImuMsg.debugData[10], mikoImuMsg.debugData[11], 
						mikoImuMsg.debugData[12], mikoImuMsg.debugData[13], mikoImuMsg.debugData[14], 
						mikoImuMsg.debugData[15], mikoImuMsg.debugData[16], mikoImuMsg.debugData[17], 
						mikoImuMsg.debugData[18], mikoImuMsg.debugData[19], mikoImuMsg.debugData[20], 
						mikoImuMsg.debugData[21], mikoImuMsg.debugData[22], mikoImuMsg.debugData[23], 
						mikoImuMsg.debugData[24], mikoImuMsg.debugData[25], mikoImuMsg.debugData[26], 
						mikoImuMsg.debugData[27], mikoImuMsg.debugData[28], mikoImuMsg.debugData[29], 
						mikoImuMsg.debugData[30], mikoImuMsg.debugData[31], mikoCmdMsg.pitch,
						mikoCmdMsg.roll, mikoCmdMsg.yaw, mikoCmdMsg.throttle,
						PIDHeightMsg.p_gain, PIDHeightMsg.i_gain, PIDHeightMsg.d_gain,
						PIDHeightMsg.i_max, PIDHeightMsg.i_min, PIDHeightMsg.goal,
						PIDHeightMsg.p_error, PIDHeightMsg.i_error, PIDHeightMsg.d_error,
                        PIDHeightMsg.hover, PIDHeightMsg.pid_value, rangeMsg.range,
                        rangeMsg.header.stamp.toSec(), 
                        x_int, y_int, x_dif_vel, y_dif_vel,
                        PIDXMsg.goal, PIDXMsg.p_error, PIDXMsg.i_error,
                        PIDXMsg.d_error, PIDXMsg.pid_value,
                        PIDXSecondMsg.goal, PIDXSecondMsg.p_error, PIDXSecondMsg.i_error,
                        PIDXSecondMsg.d_error, PIDXSecondMsg.pid_value,
                        globalGoalMsg.pose.position.x, globalGoalMsg.pose.position.y,
                        localGoalMsg.x, localGoalMsg.y
                        );

        ros::WallDuration(0.02).sleep();

		ros::spinOnce();
	}
	fclose (logFile);

}


// Pose3D callback
void poseCallback (const croc_Pose3D::Pose3DConstPtr& msg)
{
	pose3DMsg = *msg;

    ros::Time current_time = ros::Time::now();
    double delta_t = (current_time - last_dif_time).toSec();
    last_dif_time = current_time;
    x_dif_vel = (pose3DMsg.x - last_pose.x)/delta_t;
    y_dif_vel = (pose3DMsg.y - last_pose.y)/delta_t;
    last_pose = pose3DMsg;
}


// Flight Controller IMU callback
void mikoImuCallback (const cyphy_serial_driver::mikoImuConstPtr& msg)
{
	mikoImuMsg = *msg;
}


// Flight Controller Commands callback
void mikoCmdCallback (const cyphy_serial_driver::mikoCmdConstPtr& msg)
{
	mikoCmdMsg = *msg;
}


// Altitude PID data callback
void PIDHeightCallback (const croc_command::PIDConstPtr& msg)
{
	PIDHeightMsg = *msg;
}


// Ultrasound ranger callback
void rangeHeightCallback (const sensor_msgs::RangeConstPtr& msg)
{
    rangeMsg = *msg;
}


// PX4FLOW Smart Camera callback
void px4flowCallback (const px_comm::OpticalFlowConstPtr& msg)
{
    optMsg = *msg;

    ros::Time current_time = ros::Time::now();
    double delta_t = (current_time - last_int_time).toSec();
    last_int_time = current_time;
    double theta = pose3DMsg.yaw;
    double abs_x_vel=optMsg.velocity_x *cos(theta) - optMsg.velocity_y * sin(theta);
    double abs_y_vel=optMsg.velocity_x * sin(theta) + optMsg.velocity_y * cos(theta);

    x_int = x_int + abs_x_vel * delta_t;
    y_int = y_int + abs_y_vel * delta_t;

}


// PID Horizontal X Position callback
void PIDXCallback (const croc_command::PIDConstPtr& msg)
{
    PIDXMsg = *msg;
}


// PID Horizontal X Speed callback
void PIDXSecondCallback (const croc_command::PIDConstPtr& msg)
{
    PIDXSecondMsg = *msg;
}


// Global Goal callback
void GlobalGoalCallback (const geometry_msgs::PoseStampedConstPtr& msg)
{
    globalGoalMsg = *msg;

}

// Local Goal callback
void LocalGoalCallback (const geometry_msgs::Pose2DConstPtr& msg)
{
    localGoalMsg = *msg;

}

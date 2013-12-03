/*********************************************************	

(c) CROC Inc 2013

This source code demonstrates navigation, take-off, landing
and visual mark detection algorithms  we’ve developed for
CROC Flying robot competition 2013 (www.robots.croc.ru ).

- croc_Pose3D - current robot postion aggregation node. 
Position data is taken from lidar (via scan-matcher node), 
ultrasonic ranger (via px-ros-pkg or rosserial) and IMU 
(via cyphy_serial_driver, but not needed really)

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/

#include "croc_Pose3D.h"



int main(int argc, char**argv)
{
    ros::init(argc, argv, "croc_Pose3D");
    ros::NodeHandle n;

    // publishers
    pub = n.advertise<croc_Pose3D::Pose3D>("Pose3D", 100);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    // subscribers
    range_height_sub = n.subscribe("ultrasound", 100, rangeHeightCallback);
    imu_sub = n.subscribe("mikoImu", 100, imuHeightCallback);
    pose2D_sub = n.subscribe("pose2D", 100, pose2DCallback);
    px4flow_sub = n.subscribe("/px4flow/opt_flow", 1000, opticalFlowCallback);

    // tf-broadcasters
    tf::TransformBroadcaster odom_broadcaster;  // odometry tf broadcater for navigation stack. For some reason cannot define befor ros::init

    // get params
    if (!n.getParam("/croc_Pose3D/velocity_lpf_factor", velocity_lpf_factor)) velocity_lpf_factor = 0.3;
    if (!n.getParam("/croc_Pose3D/hangout_handling_mode", hangout_handling_mode)) hangout_handling_mode = 1;
    if (!n.getParam("/croc_Pose3D/hangout_handling_min_altitude", hangout_handling_min_altitude)) hangout_handling_min_altitude = 60;


    // inintial values
    msg.x=0.;
    msg.y=0.;
    msg.z=0.;
    msg.pitch=0.;
    msg.roll=0.;
    msg.yaw=0.;

    current_v_z = 0.0;
    prev_z = 0.0;
    prev_prev_z = 0.0;
    isVZFirstRun = true;

    of_last_time = ros::Time::now();
  
    while (ros::ok())
    {
        ros::WallDuration(0.02).sleep();
        ros::Time current_time = ros::Time::now();

        //prepare rotation quternion
        tf::Quaternion tq;      // tf-Q is easy to set with pitch-roll-yaw values
        tq.setRPY(0,0,msg.yaw); // no need in pitch and roll in 2d-navigation
        geometry_msgs::Quaternion gq;   // geometry-Q is needed for messages
        tf::quaternionTFToMsg(tq,gq);   // transform tf-Q to geometry-Q

        // publish transform odom
        // headers
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "/odom";
        odom_trans.child_frame_id = "/base_link";
       // x-y-z
        odom_trans.transform.translation.x = msg.x;
        odom_trans.transform.translation.y = msg.y;
        odom_trans.transform.translation.z = 0.0;   // no need in z for 2d-navigation stack
        // rotation
        odom_trans.transform.rotation = gq;
        // publish transform
        odom_broadcaster.sendTransform(odom_trans);


        // publish msg to odom topic odom
        nav_msgs::Odometry odom;
        // headers
        odom.header.stamp = current_time;
        odom.header.frame_id = "/odom";
        // position
        odom.pose.pose.orientation = gq;
        odom.pose.pose.position.x = msg.x;
        odom.pose.pose.position.y = msg.y;
        odom.pose.pose.position.z = 0.0; // no need in z for 2d-navigation stack

        // publish
        odom_pub.publish(odom);
        pub.publish(msg);

        ros::spinOnce();
    }

        return 0;
}



// ultrasound range callback
void rangeHeightCallback (const sensor_msgs::RangeConstPtr& rangeMsg)
{

    msg.z=rangeMsg->range;

    // calculate linear z-velocity
    calculateLinearZVelocity(rangeMsg->header.stamp);
}


// mkopter imu callback
void imuHeightCallback (const cyphy_serial_driver::mikoImuConstPtr& imuMsg)
{
    msg.pitch=imuMsg->anglePitch;
    msg.roll=imuMsg->angleRoll;
    msg.linear_acceleration = imuMsg->linear_acceleration;
}


// scan matcher pose2D callback
void pose2DCallback (const geometry_msgs::Pose2DConstPtr& pose2DMsg)
{
   msg.x = pose2DMsg->x;
   msg.y = pose2DMsg->y;
   msg.yaw = pose2DMsg->theta;
}

// px4flow optical flow callback
void opticalFlowCallback (const px_comm::OpticalFlowConstPtr optMsg)
{
    // velocity integration for position calc
    ros::Time of_current_time = ros::Time::now();
    double delta_t = (of_current_time - of_last_time).toSec();

    of_last_time = of_current_time;

    int fraction_factor = 1;
    double measuredDistance = optMsg->ground_distance*100;

    //check for hangout
    if (measuredDistance >= hangout_handling_min_altitude)
    {
    switch(hangout_handling_mode)
        {
            // dividing by 2 or 3 handling
            case 1:
		if (measuredDistance>prev_measured*3.75)
                {
                    fraction_factor = 4;
                    measuredDistance =  measuredDistance / fraction_factor; // - 3.2 * fraction_factor;
					// measuredDistance = (measuredDistance - 30) / fraction_factor;
                    ROS_INFO("handling mode 1: factor *4   %f   %f", measuredDistance, prev_measured);

                } 
		else if (measuredDistance>prev_measured*2.5)
                {
                    fraction_factor = 3;
                    measuredDistance =  measuredDistance / fraction_factor; // - 3.2 * fraction_factor;
					// measuredDistance = (measuredDistance - 30) / fraction_factor;
                    ROS_INFO("handling mode 1: factor *3   %f   %f", measuredDistance, prev_measured);

                }
                else if(measuredDistance>prev_measured*1.75)
                {
                    fraction_factor= 2;
					measuredDistance =  measuredDistance / fraction_factor; //  - 3.2 * fraction_factor;
                    //measuredDistance = (measuredDistance - 15)/fraction_factor;
                    ROS_INFO("handling mode 1: factor *2   %f   %f", measuredDistance, prev_measured);
                }
                else prev_measured = measuredDistance;
                break;
             // taking prev measurement
             case 2:
                if(measuredDistance>prev_measured*1.75)
                {
                    prev_measured = measuredDistance;
                    ROS_INFO("handling mode 2:  %f   %f", measuredDistance, prev_measured);
                    measuredDistance = msg.z;

                }
                break;
        }
    } 
    else 
    {
        prev_measured = measuredDistance;
    }


    msg.calculated_linear_velocity.x = (1 - velocity_lpf_factor) * msg.calculated_linear_velocity.x + velocity_lpf_factor * optMsg->velocity_x / fraction_factor;
    msg.calculated_linear_velocity.y = (1 - velocity_lpf_factor) * msg.calculated_linear_velocity.y + velocity_lpf_factor * optMsg->velocity_y / fraction_factor;
 

    double a = 0.0; //0.5;
    msg.z=prev_z * a + measuredDistance * (1-a);

    // calculate linear z-velocity
    calculateLinearZVelocity(of_current_time); //optMsg->header.stamp);

}


// calculate z-velocity based on sonar data
void calculateLinearZVelocity(ros::Time stamp)
{
    if (isVZFirstRun)
    {
        prev_z = msg.z;
        prev_prev_z = msg.z;
        prev_z_time = stamp;
        prev_prev_z_time = stamp;
        current_v_z = 0;
        isVZFirstRun = false;
    }
    else
    {
        if (prev_z!=msg.z)
        {
            current_v_z = (msg.z-prev_prev_z)/(stamp - prev_prev_z_time).toSec();
            prev_prev_z = prev_z;
            prev_prev_z_time = prev_z_time;
            prev_z = msg.z;
            prev_z_time = stamp;
            msg.calculated_linear_velocity.z = current_v_z;
        } 
    }
}

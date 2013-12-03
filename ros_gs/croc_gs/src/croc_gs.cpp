/*********************************************************	

(c) CROC Inc 2013

This source code demonstrates navigation, take-off, landing
and visual mark detection algorithms  we’ve developed for
CROC Flying robot competition 2013 (www.robots.croc.ru ).

croc_gs - Ground Station is a small software, based on qt4
& ROS rviz package for control robot and getting some
telemetry from it (position, speed, video, etc). Also it has
some wind control - for gazebo emulation purposes only.

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/

#include <QGridLayout>

#include <QPushButton>
#include <QGroupBox>
#include <QCheckBox>
#include <QLabel>
#include <QTextEdit>
#include <QLineEdit>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include "croc_gs.h"

#include <geometry_msgs/PoseStamped.h>
#include <State.h>
#include <Pose3D.h>
#include <mikoCmd.h>
#include <rosgraph_msgs/Log.h>

// Constructor
CrocGS::CrocGS( QWidget* parent )
  : QWidget( parent )
{

    // ros node init
    ros::NodeHandle n;
    globalGoalPub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 50);
    commandStatePub = n.advertise<croc_command::State>("/StateCmd", 50);
    windPub = n.advertise<croc_gs::windParameters>("/WindParameters", 50);
    pose3dSub = n.subscribe("Pose3D", 100, &CrocGS::poseCallback, this );
    cmdSub = n.subscribe("c_command", 100, &CrocGS::cmdCallback, this );
    stateSub = n.subscribe("StateCurrent", 100, &CrocGS::stateCallback, this );
    statecmdSub = n.subscribe("StateCmd", 100, &CrocGS::stateCallback, this );
    detectingSub = n.subscribe("DetectingMark", 100, &CrocGS::detectingCallback, this );
    landingSub = n.subscribe("LandingFieldPos", 100, &CrocGS::landingCallback, this );
    logSub = n.subscribe("rosout", 100, &CrocGS::rosoutCallback, this );

	// layouts init
    QGridLayout* controls_layout = new QGridLayout();

    // start-land buttons
    QPushButton* start_button = new QPushButton("Start");
    QPushButton* land_button = new QPushButton("Land");
    QPushButton* crit_land_button = new QPushButton("Critical Land");
    QPushButton* turn_off_engines = new QPushButton("Off Engines");
    label_flightTimer = new QLabel();
    controls_layout->addWidget(start_button, 0, 0);
    controls_layout->addWidget(land_button, 0, 1);
    controls_layout->addWidget(crit_land_button, 0, 2);
    controls_layout->addWidget(turn_off_engines, 0, 3);
    controls_layout->addWidget(label_flightTimer, 0, 4);
    connect(start_button, SIGNAL(clicked()), this, SLOT(startQuad()));
    connect(land_button, SIGNAL(clicked()), this, SLOT(landQuad()));
    connect(crit_land_button, SIGNAL(clicked()), this, SLOT(criticallandQuad()));
    connect(turn_off_engines, SIGNAL(clicked()), this, SLOT(offQuad()));
    flightTimer = new QTimer(this);
    connect(flightTimer, SIGNAL(timeout()), this, SLOT(flightTimerUpdate()));


    // display checkboxes
    QGridLayout* displays_control_layout = new QGridLayout();
    QGroupBox* displays_group = new QGroupBox("Displays on/off");
    QCheckBox *checkbox_costmap = new QCheckBox(tr("&Costmap"));
    QCheckBox *checkbox_tf = new QCheckBox(tr("&TF"));
    QCheckBox *checkbox_path = new QCheckBox(tr("&Path"));
    QCheckBox *checkbox_laser = new QCheckBox(tr("&Laser"));
    QCheckBox *checkbox_video = new QCheckBox(tr("&Video"));
    QCheckBox *checkbox_wind = new QCheckBox(tr("&Wind"));
    checkbox_costmap->setChecked(true);
    checkbox_tf->setChecked(true);
    checkbox_path->setChecked(true);
    checkbox_laser->setChecked(false);
    checkbox_wind->setChecked(true);
    QGridLayout* displays_checkboxes_layout = new QGridLayout();
    displays_checkboxes_layout->addWidget(checkbox_costmap, 0, 0);
    displays_checkboxes_layout->addWidget(checkbox_tf, 0, 1);
    displays_checkboxes_layout->addWidget(checkbox_path, 1, 0);
    displays_checkboxes_layout->addWidget(checkbox_laser, 1, 1);
    displays_checkboxes_layout->addWidget(checkbox_video, 0, 2);
    displays_checkboxes_layout->addWidget(checkbox_wind, 1, 2);
    displays_group->setLayout(displays_checkboxes_layout);
    displays_control_layout->addWidget(displays_group);
    connect(checkbox_tf, SIGNAL(clicked(bool)), this, SLOT(checkbox_TF_click(bool)));
    connect(checkbox_costmap, SIGNAL(clicked(bool)), this, SLOT(checkbox_Costmap_click(bool)));
    connect(checkbox_laser, SIGNAL(clicked(bool)), this, SLOT(checkbox_Laser_click(bool)));
    connect(checkbox_path, SIGNAL(clicked(bool)), this, SLOT(checkbox_Path_click(bool)));
    connect(checkbox_video, SIGNAL(clicked(bool)), this, SLOT(checkbox_Video_click(bool)));
    connect(checkbox_wind, SIGNAL(clicked(bool)), this, SLOT(checkbox_Wind_click(bool)));


    // telemetry
    QGridLayout* telemetry_layout = new QGridLayout();
    label_pose3d = new QLabel();
    label_pose3d_x = new QLabel();
    label_pose3d_y = new QLabel();
    label_pose3d_z = new QLabel();
    label_pose3d_pitch = new QLabel();
    label_pose3d_roll = new QLabel();
    label_pose3d_yaw = new QLabel();
    label_pose3d_cvel_x = new QLabel();
    label_pose3d_cvel_y = new QLabel();
    label_pose3d_cvel_z = new QLabel();

    label_c_command = new QLabel();
    label_c_command_pitch = new QLabel();
    label_c_command_roll = new QLabel();
    label_c_command_yaw = new QLabel();
    label_c_command_throttle = new QLabel();

    label_state = new QLabel();
    label_state_cmd = new QLabel();

    label_detecting = new QLabel();
    label_detecting_x = new QLabel();
    label_detecting_y = new QLabel();
    label_detecting_ground_x = new QLabel();
    label_detecting_ground_y = new QLabel();


    telemetry_layout->addWidget(label_pose3d,0,0);
    telemetry_layout->addWidget(label_pose3d_x,1,0);
    telemetry_layout->addWidget(label_pose3d_y,2,0);
    telemetry_layout->addWidget(label_pose3d_z,3,0);
    telemetry_layout->addWidget(label_pose3d_pitch,1,1);
    telemetry_layout->addWidget(label_pose3d_roll,2,1);
    telemetry_layout->addWidget(label_pose3d_yaw,3,1);
    telemetry_layout->addWidget(label_pose3d_cvel_x,1,2);
    telemetry_layout->addWidget(label_pose3d_cvel_y,2,2);
    telemetry_layout->addWidget(label_pose3d_cvel_z,3,2);

    telemetry_layout->addWidget(label_c_command,4,0);
    telemetry_layout->addWidget(label_c_command_pitch,5,0);
    telemetry_layout->addWidget(label_c_command_roll,5,1);
    telemetry_layout->addWidget(label_c_command_yaw,6,0);
    telemetry_layout->addWidget(label_c_command_throttle,6,1);

    telemetry_layout->addWidget(label_state,7,0);
    telemetry_layout->addWidget(label_state_cmd,8,0);

    telemetry_layout->addWidget(label_detecting,9,0);
    telemetry_layout->addWidget(label_detecting_x,10,0);
    telemetry_layout->addWidget(label_detecting_y,11,0);
    telemetry_layout->addWidget(label_detecting_ground_x,10,1);
    telemetry_layout->addWidget(label_detecting_ground_y,11,1);


    label_pose3d->setText("Pose3D");
    label_c_command->setText("Commands");
    label_state->setText("State");
    label_detecting->setText("Detecting");

    QFont qfont = label_pose3d->font();
    qfont.setBold(true);
    label_pose3d->setFont(qfont);
    label_c_command->setFont(qfont);
    label_state->setFont(qfont);
    label_detecting->setFont(qfont);


    telemetry_layout->setColumnStretch(0,0);
    telemetry_layout->setColumnStretch(1,0);
    telemetry_layout->setColumnStretch(2,1);
    telemetry_layout->setRowStretch(0,0);
    telemetry_layout->setRowStretch(1,0);
    telemetry_layout->setRowStretch(2,0);
    telemetry_layout->setRowStretch(3,0);
    telemetry_layout->setRowStretch(4,0);
    telemetry_layout->setRowStretch(5,0);
    telemetry_layout->setRowStretch(6,0);
    telemetry_layout->setRowStretch(7,0);
    telemetry_layout->setRowStretch(8,0);
    telemetry_layout->setRowStretch(9,0);
    telemetry_layout->setRowStretch(10,0);
    telemetry_layout->setRowStretch(11,0);
    telemetry_layout->setRowStretch(12,1);

    telemetry_layout->setColumnMinimumWidth(0,150);
    telemetry_layout->setColumnMinimumWidth(1,150);
    telemetry_layout->setColumnMinimumWidth(2,150);

	// text log
    text_log = new QTextEdit();

    // RVIZ panel
    render_panel_ = new rviz::RenderPanel();
    QGridLayout* main_layout = new QGridLayout;
    QGridLayout* left_layout = new QGridLayout;
    QGridLayout* right_layout = new QGridLayout;

    left_layout->addLayout( controls_layout, 0, 0 );
    left_layout->addWidget( render_panel_, 1, 0 );
    left_layout->addLayout( displays_control_layout, 2, 0 );
    right_layout->addLayout( telemetry_layout, 0, 0 );
    right_layout->addWidget( text_log, 1, 0 );
    main_layout->addLayout( left_layout, 0, 0 );
    main_layout->addLayout( right_layout, 0, 1 );

    left_layout->setRowStretch(0,0);
    left_layout->setRowStretch(1,1);
    left_layout->setRowStretch(2,0);
    right_layout->setRowStretch(0,0);
    right_layout->setRowStretch(1,1);
    main_layout->setColumnStretch(0,1);
    main_layout->setColumnStretch(1,0);

    // Set the top-level layout
    setLayout( main_layout );

    // Initialize RViz
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    // Create displays.
    grid_ = manager_->createDisplay( "rviz/Grid", "Grid", true );
    ROS_ASSERT( grid_ != NULL );
    costmap_ = manager_->createDisplay("rviz/GridCells", "Global Costmap", true);
    ROS_ASSERT( costmap_ != NULL );
    tf_ = manager_->createDisplay("rviz/TF", "TF", true);
    ROS_ASSERT( tf_ != NULL );
    laser_= manager_->createDisplay("rviz/LaserScan", "LaserScan", true);
    ROS_ASSERT( laser_ != NULL );
    path_= manager_->createDisplay("rviz/Path", "Path", true);
    ROS_ASSERT( path_ != NULL );
    wind_ = manager_->createDisplay("rviz/WrenchStamped", "Wind", true);
    ROS_ASSERT( wind_ != NULL );
    video_= manager_->createDisplay("rviz/Image", "Image", true);
    ROS_ASSERT( video_ != NULL );
    detecting_video_= manager_->createDisplay("rviz/Image", "DetectingImage", true);
    ROS_ASSERT( detecting_video_ != NULL );
    intersection_video_= manager_->createDisplay("rviz/Image", "IntersectionImage", true);
    ROS_ASSERT( intersection_video_ != NULL );
    threshold_video_= manager_->createDisplay("rviz/Image", "ThresholdImage", true);
    ROS_ASSERT( threshold_video_ != NULL );
    right_layout->addWidget( video_->getAssociatedWidget(), 2, 0 );


    // Configure displays
    grid_->subProp( "Line Style" )->setValue( "Billboards" );
    grid_->subProp( "Color" )->setValue( Qt::gray );
    costmap_->subProp("Topic")->setValue("/move_base/global_costmap/obstacles");
    costmap_->subProp("Color")->setValue(Qt::green);
    laser_->subProp("Topic")->setValue("/scan");
    laser_->subProp("Color")->setValue(Qt::red);
    laser_->setEnabled(false);
    path_->subProp("Topic")->setValue("/move_base/NavfnROS/plan");
    path_->subProp("Color")->setValue(Qt::yellow);
    wind_->subProp("Topic")->setValue("/Wind");
    wind_->subProp("Arrow Scale")->setValue("1");
    video_->subProp("Image Topic")->setValue("/ros_camera_plugin_node/croc_image");
    video_->setEnabled(false);
    detecting_video_->subProp("Image Topic")->setValue("/DetectingImage");
    detecting_video_->setEnabled(false);
    intersection_video_->subProp("Image Topic")->setValue("/IntersectionImage");
    intersection_video_->setEnabled(false);
    threshold_video_->subProp("Image Topic")->setValue("/ThresholdImage");
    threshold_video_->setEnabled(false);

    detecting_video_->subProp("Transport Hint")->setValue("compressed");
    intersection_video_->subProp("Transport Hint")->setValue("compressed");
    threshold_video_->subProp("Transport Hint")->setValue("compressed");

    // Wind params
    wind_layout = new QGridLayout();
    edit_basevectorX = new QLineEdit("1");
    edit_basevectorY = new QLineEdit("1");
    edit_alphaMin = new QLineEdit("5");
    edit_alphaMax = new QLineEdit("15");
    edit_betaMin = new QLineEdit("1");
    edit_betaMax = new QLineEdit("2");
    edit_gammaMin = new QLineEdit("0.1");
    edit_gammaMax = new QLineEdit("0.2");
    edit_kappaMin = new QLineEdit("1");
    edit_kappaMax = new QLineEdit("2");
    QPushButton* button_updateWind = new QPushButton("Update Wind");
    QLabel* label_basevector = new QLabel("Wind vector");
    QLabel* label_alpha = new QLabel("Min/Max Angle");
    QLabel* label_beta = new QLabel("Min/Max Freq.");
    QLabel* label_gamma = new QLabel("Min/Max Strength");
    QLabel* label_kappa = new QLabel("Min/Max Str. Freq.");

    wind_layout->addWidget( label_basevector, 0, 0 );
    wind_layout->addWidget( edit_basevectorX, 0, 1 );
    wind_layout->addWidget( edit_basevectorY, 0, 2 );
    wind_layout->addWidget( button_updateWind, 0, 3 );

    wind_layout->addWidget( label_alpha, 1, 0 );
    wind_layout->addWidget( label_beta, 1, 1 );
    wind_layout->addWidget( label_gamma, 1, 2 );
    wind_layout->addWidget( label_kappa, 1, 3 );

    wind_layout->addWidget( edit_alphaMin, 2, 0 );
    wind_layout->addWidget( edit_alphaMax, 3, 0 );
    wind_layout->addWidget( edit_betaMin, 2, 1 );
    wind_layout->addWidget( edit_betaMax, 3, 1 );
    wind_layout->addWidget( edit_gammaMin, 2, 2 );
    wind_layout->addWidget( edit_gammaMax, 3, 2 );
    wind_layout->addWidget( edit_kappaMin, 2, 3 );
    wind_layout->addWidget( edit_kappaMax, 3, 3 );

    main_layout->addLayout( wind_layout, 3, 0 );

    connect(button_updateWind, SIGNAL(clicked()), this, SLOT(updateWind()));


    // Detecting Img
    QGridLayout* detectingimage_layout = new QGridLayout();
    detectingimage_layout->addWidget( detecting_video_->getAssociatedWidget(), 0, 0 );
    detectingimage_layout->addWidget( threshold_video_->getAssociatedWidget(), 0, 1 );
    detectingimage_layout->addWidget( intersection_video_->getAssociatedWidget(), 0, 2 );
    main_layout->addLayout( detectingimage_layout, 3, 1 );

}


// Destructor.
CrocGS::~CrocGS()
{
  delete manager_;
}


// SIGNAL handlers


// Start Button handler
void  CrocGS::startQuad()
{

    // set global goal to 2;0. Historically for navi node it's equal
	// to start engines, take off and start contest task. (Because it
	// was easy to use rviz for start robot that way)
    geometry_msgs::PoseStamped goalMsg;
    goalMsg.pose.position.x=2;
    goalMsg.pose.position.y=0;
    goalMsg.pose.orientation.w=1;
    goalMsg.header.frame_id = "world";
    goalMsg.header.stamp = ros::Time::now();
    globalGoalPub.publish(goalMsg);

    flightTimer->start(500);
    takeoffTime=ros::Time::now();

}


// Land button handler
void  CrocGS::landQuad()
{
    croc_command::State stateMsg;
    stateMsg.state = 4;     // Landing
    commandStatePub.publish(stateMsg);
}


// Critical Land button handler
void  CrocGS::criticallandQuad()
{
    croc_command::State stateMsg;
    stateMsg.state = 5;     // Extreme Landing
    commandStatePub.publish(stateMsg);
}


// Off engines button handler
void  CrocGS::offQuad()
{
    croc_command::State stateMsg;
    stateMsg.state = 0;     // Ground
    commandStatePub.publish(stateMsg);
}



// update wind button handler
void CrocGS::updateWind()
{
 	// get data from fields and send it to WindParrams topic
	croc_gs::windParameters msg;
    msg.baseVector.x = edit_basevectorX->text().toFloat();
    msg.baseVector.y = edit_basevectorY->text().toFloat();

    msg.maxAlpha = edit_alphaMax->text().toFloat();
    msg.minAlpha = edit_alphaMin->text().toFloat();
    msg.maxBeta = edit_betaMax->text().toFloat();
    msg.minBeta = edit_betaMin->text().toFloat();
    msg.maxGamma = edit_gammaMax->text().toFloat();
    msg.minGamma = edit_gammaMin->text().toFloat();
    msg.maxKappa = edit_kappaMax->text().toFloat();
    msg.minKappa = edit_kappaMin->text().toFloat();

    windPub.publish(msg);
}


// Flight timer update
void CrocGS::flightTimerUpdate()
{
	// print time passing from takeoff.
	int minutes, sec;
    sec = (int)(ros::Time::now()-takeoffTime).toSec()%60;
    minutes = (int)(ros::Time::now()-takeoffTime).toSec()/60;
    label_flightTimer->setText("Flight Time: " + QString::number(minutes)+ ":" + QString::number(sec));
}


// checkbox on\off path display handler
void CrocGS::checkbox_Path_click(bool value)
{
    path_->setEnabled(value);
}

// checkbox on\off TF display handler
void CrocGS::checkbox_TF_click(bool value)
{
    tf_->setEnabled(value);
}

// checkbox on\off laser scan display handler
void CrocGS::checkbox_Laser_click(bool value)
{
    laser_->setEnabled(value);
}

// checkbox on\off costmap display handler
void CrocGS::checkbox_Costmap_click(bool value)
{
    costmap_->setEnabled(value);
}

// checkbox on\off video display handler
void CrocGS::checkbox_Video_click(bool value)
{
    video_->setEnabled(value);
    detecting_video_->setEnabled(value);
    intersection_video_->setEnabled(value);
    threshold_video_->setEnabled(value);
}

// checkbox on\off wind display handler
void CrocGS::checkbox_Wind_click(bool value)
{
    wind_->setEnabled(value);
}


// Callbacks for telemetry data


// callback for Pose3D data
void CrocGS::poseCallback (const croc_Pose3D::Pose3DConstPtr& msg)
{
    label_pose3d_x->setText("X: " + QString::number(msg->x));
    label_pose3d_y->setText("Y: " + QString::number(msg->y));
    label_pose3d_z->setText("Z: " + QString::number(msg->z));

    label_pose3d_pitch->setText("Pitch: " + QString::number(msg->pitch));
    label_pose3d_roll->setText("Roll: " + QString::number(msg->roll));
    label_pose3d_yaw->setText("Yaw: " + QString::number(msg->yaw));

    label_pose3d_cvel_x->setText("VelX: " + QString::number(msg->calculated_linear_velocity.x));
    label_pose3d_cvel_y->setText("VelY: " + QString::number(msg->calculated_linear_velocity.y));
    label_pose3d_cvel_z->setText("VelZ: " + QString::number(msg->calculated_linear_velocity.z));

}


// callback for flight controller command data
void CrocGS::cmdCallback (const cyphy_serial_driver::mikoCmdConstPtr msg)
{
    label_c_command_pitch->setText("CmdPitch: " + QString::number(msg->pitch));
    label_c_command_roll->setText("CmdRoll: " + QString::number(msg->roll));
    label_c_command_yaw->setText("CmdYaw: " + QString::number(msg->yaw));
    label_c_command_throttle->setText("CmdThrottle: " + QString::number(msg->throttle));
}


// callback for physical state data
void CrocGS::stateCallback (const croc_command::StateConstPtr msg)
{

    QString value;
    switch (msg->state)
    {
       case GROUND:
        value = "Ground";
        break;
       case STARTING_ENGINES:
        value = "Starting Engines";
        break;
       case TAKEOFF:
        value = "Taking off";
        break;
       case AIR:
        value = "In the air";
        break;
       case LANDING:
        value = "Landing";
        break;
     }

    label_state_cmd->setText("CmdState: " + value);

}


// callback for landing mark image detecting data
void CrocGS::detectingCallback (const geometry_msgs::Pose2DConstPtr msg)
{
    label_detecting_x->setText("ImgX:" + QString::number(msg->x));
    label_detecting_y->setText("ImgY:" + QString::number(msg->y));
}


// callback for landing mark position data
void CrocGS::landingCallback (const geometry_msgs::Pose2DConstPtr msg)
{
    label_detecting_ground_x->setText("GroundX:" + QString::number(msg->x));
    label_detecting_ground_y->setText("GroundY:" + QString::number(msg->y));
}

// callback for text log data
void CrocGS::rosoutCallback(const rosgraph_msgs::LogConstPtr msg)
{
   text_log->append(QString::fromStdString(msg->msg));
}

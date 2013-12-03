#include <QWidget>
#include <QGridLayout>
#include <QLabel>
#include <QTextEdit>
#include <QLineEdit>
#include <QTimer>

#include <ros/ros.h>
#include <Pose3D.h>
#include <mikoCmd.h>
#include <State.h>
#include <geometry_msgs/Pose2D.h>
#include <rosgraph_msgs/Log.h>
#include "croc_gs/windParameters.h"

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}


class CrocGS: public QWidget
{
Q_OBJECT
public:
  CrocGS( QWidget* parent = 0 );
  virtual ~CrocGS();

private Q_SLOTS:
  void startQuad();
  void landQuad();
  void criticallandQuad();
  void offQuad();
  void updateWind();
  void checkbox_Path_click(bool value);
  void checkbox_TF_click(bool value);
  void checkbox_Laser_click(bool value);
  void checkbox_Costmap_click(bool value);
  void checkbox_Video_click(bool value);
  void checkbox_Wind_click(bool value);
  void flightTimerUpdate();


private:
  rviz::VisualizationManager* manager_;
  rviz::RenderPanel* render_panel_;
  rviz::Display* grid_;
  rviz::Display* costmap_;
  rviz::Display* tf_;
  rviz::Display* path_;
  rviz::Display* laser_;
  rviz::Display* video_;
  rviz::Display* detecting_video_;
  rviz::Display* intersection_video_;
  rviz::Display* threshold_video_;
  rviz::Display* wind_;


  ros::Publisher globalGoalPub;
  ros::Publisher commandStatePub;
  ros::Publisher windPub;

  ros::Subscriber pose3dSub;
  ros::Subscriber cmdSub;
  ros::Subscriber stateSub;
  ros::Subscriber statecmdSub;
  ros::Subscriber detectingSub;
  ros::Subscriber landingSub;
  ros::Subscriber logSub;


  // telemetry labels
  QLabel* label_pose3d;
  QLabel* label_pose3d_x;
  QLabel* label_pose3d_y;
  QLabel* label_pose3d_z;
  QLabel* label_pose3d_pitch;
  QLabel* label_pose3d_roll;
  QLabel* label_pose3d_yaw;
  QLabel* label_pose3d_cvel_x;
  QLabel* label_pose3d_cvel_y;
  QLabel* label_pose3d_cvel_z;

  QLabel* label_c_command;
  QLabel* label_c_command_pitch;
  QLabel* label_c_command_roll;
  QLabel* label_c_command_yaw;
  QLabel* label_c_command_throttle;

  QLabel* label_state;
  QLabel* label_state_cmd;


  QLabel* label_detecting;
  QLabel* label_detecting_x;
  QLabel* label_detecting_y;
  QLabel* label_detecting_ground_x;
  QLabel* label_detecting_ground_y;


  QGridLayout* wind_layout;
  QLineEdit* edit_basevectorX;
  QLineEdit* edit_basevectorY;
  QLineEdit* edit_alphaMin;
  QLineEdit* edit_alphaMax;
  QLineEdit* edit_betaMin;
  QLineEdit* edit_betaMax;
  QLineEdit* edit_gammaMin;
  QLineEdit* edit_gammaMax;
  QLineEdit* edit_kappaMin;
  QLineEdit* edit_kappaMax;

  QTextEdit* text_log;

  QTimer* flightTimer;
  QLabel* label_flightTimer;
  ros::Time takeoffTime;


  // telemetry data callbacks
  void poseCallback (const croc_Pose3D::Pose3DConstPtr& msg);
  void cmdCallback (const cyphy_serial_driver::mikoCmdConstPtr msg);
  void stateCallback (const croc_command::StateConstPtr msg);
  void detectingCallback (const geometry_msgs::Pose2DConstPtr msg);
  void landingCallback (const geometry_msgs::Pose2DConstPtr msg);
  void rosoutCallback (const rosgraph_msgs::LogConstPtr msg);

  // physical states enum
  enum {GROUND, STARTING_ENGINES, TAKEOFF, AIR, LANDING, EXTREME_LANDING};

};


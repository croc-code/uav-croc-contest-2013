#include <QApplication>
#include <ros/ros.h>
#include "croc_gs.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized() )
  {
    ros::init( argc, argv, "croc_gs", ros::init_options::AnonymousName );
  }

  QApplication app( argc, argv );

  CrocGS* crocgs = new CrocGS();
  crocgs->show();

  app.exec();

  delete crocgs;
}

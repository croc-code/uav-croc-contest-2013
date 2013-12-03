#include "math/Vector3.hh"
#include "ros/ros.h"

namespace croc
{
  class Wind
  {
  public:

      Wind();


    ~Wind ();

      // constructor
      // baseVector - base wind direction
      // min\max Alpha - limits of force direction (in degrees)
      // min\max Beta  - limits of force direction changing period (in seconds)
      // min\max Gamma - limits of force strength
      // min\max Kappa - limits of force strength changing period (in seconds)
     void initWind (gazebo::math::Vector3 baseVector,
            double minAlpha, double maxAlpha,
            double minBeta, double maxBeta,
            double minGamma, double maxGamma,
            double minKappa, double maxKappa);

    gazebo::math::Vector3 getCurrentWind();


    void updateValues();

 private:

   double getRandom(double min, double max);

   double changeCoefs();

   // parameters struct
   // baseVector - base wind direction
   // min\max Alpha - limits of force direction (in degrees)
   // min\max Beta  - limits of force direction changing period (in seconds)
   // min\max Gamma - limits of force strength
   // min\max Kappa - limits of force strength changing period (in seconds)
   struct parameters_t{
       gazebo::math::Vector3 baseVector;
       double minAlpha;
       double maxAlpha;
       double minBeta;
       double maxBeta;
       double minGamma;
       double maxGamma;
       double minKappa;
       double maxKappa;
   } params;

   // angle of wind variables
   double alpha_v;
   double beta_v;
   double alpha_h;
   double beta_h;
   double psi;
   double phi;

   // strength of wind variables
   double gamma;
   double kappa;

   // time
   ros::Time currentTime;
   ros::Time startingTime;
  };
}

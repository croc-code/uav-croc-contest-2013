/*********************************************************

(c) CROC Inc 2013

This source code demonstrates gazebosim 1.9 plugins we used
for testing our solution for CROC Flying robot competition
2013 (www.robots.croc.ru ).

gazebo_flyer_plugin - flight controller realization for
gazebo robot model. It emulates Mikrokopter FK and
cyphy_serial_driver node: it gets target pitch, roll and yaw
and try to change model pose to this PRY values with four
forces which emulate propellers.

wind.cpp - object, representing wind as linear force which
related to object (robot)

Source code can be used with no charge for any non-commercial
purposes. Feel free to ask us any questions at robots@croc.ru .

**********************************************************/

#include "include/wind.h"
#include "ros/ros.h"


namespace croc
{

    Wind::Wind()
    {}

    Wind::~Wind ()
    {}

    // constructor
    // baseVector - base wind direction
    // min\max Alpha - limits of force direction (in degrees)
    // min\max Beta  - limits of force direction changing period (in seconds)
    // min\max Gamma - limits of force strength
    // min\max Kappa - limits of force strength changing period (in seconds)
    void Wind::initWind (gazebo::math::Vector3 baseVector,
            double minAlpha, double maxAlpha,
            double minBeta, double maxBeta,
            double minGamma, double maxGamma,
            double minKappa, double maxKappa)
    {
        startingTime = ros::Time::now();

        params.baseVector = baseVector;
        params.minAlpha = minAlpha;
        params.maxAlpha = maxAlpha;
        params.minBeta = minBeta;
        params.maxBeta = maxBeta;
        params.minGamma = minGamma;
        params.maxGamma = maxGamma;
        params.minKappa = minKappa;
        params.maxKappa = maxKappa;

        alpha_v = getRandom(minAlpha, maxAlpha);
        alpha_h = getRandom(minAlpha, maxAlpha);
        beta_v = getRandom(minBeta, maxBeta);
        beta_h = getRandom(minBeta, maxBeta);
        gamma = getRandom(minGamma, maxGamma);
        kappa = getRandom(minKappa, maxKappa);
    }


    void Wind::updateValues()
    {
        double delta_t = (ros::Time::now() - startingTime).toSec();

    }


    // get current wind force value based on current values and time
    gazebo::math::Vector3 Wind::getCurrentWind()
    {
        gazebo::math::Vector3 result;
        double delta_t = (ros::Time::now() - startingTime).toSec();

        // angles of force (vertical & horizontal)
        phi = alpha_v * sin(beta_v * delta_t);
        psi = alpha_h * sin(beta_h * delta_t);
        phi = phi * M_PI / 180;
        psi = psi * M_PI / 180;

        // calculate wind force value
        double L = sqrt(pow(params.baseVector.x,2) + pow(params.baseVector.y,2));
        result.x = params.baseVector.x * cos(phi) - params.baseVector.y * sin(phi);
        result.y = params.baseVector.x * sin(phi) - params.baseVector.y * cos(phi);
        result.z = L*tan(psi);
        result.Normalize();
        double strength = gamma * (sin(kappa * delta_t)+1);
        result = result * strength;

        changeCoefs();

        return result;

    }


    // when periods pass - reinit coefficients
    double Wind::changeCoefs()
    {
        double delta_t = (ros::Time::now() - startingTime).toSec();

        if (fmod(beta_v * delta_t, 2*M_PI)<0.1) {beta_v = getRandom(params.minBeta, params.maxBeta); alpha_v = getRandom(params.minAlpha, params.maxAlpha); }
        if (fmod(beta_h * delta_t, 2*M_PI)<0.1) {beta_h = getRandom(params.minBeta, params.maxBeta); alpha_h = getRandom(params.minAlpha, params.maxAlpha); }
        if (fmod(kappa * delta_t, 2*M_PI)<0.1) {kappa = getRandom(params.minKappa, params.maxKappa); gamma = getRandom(params.minGamma, params.maxGamma); }

    }


    // random value between min and max values
    double Wind::getRandom(double min, double max)
    {
        double f = (double)rand() / RAND_MAX;
        return min + f * (max - min);
    }

}


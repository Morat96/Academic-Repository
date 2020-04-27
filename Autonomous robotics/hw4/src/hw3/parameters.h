#ifndef ENVCONST
#define ENVCONST

#include <ros/ros.h>
#include <stdlib.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <std_srvs/Empty.h>

using namespace std;
class parameters {

public:

    /**
     * @brief set DWA Algorithm parameters
     */
	static void setDWAparameters();

    /**
     * @brief update AMCL particles
     */
	static void updateAMCLParticles(ros::NodeHandle n);
};

#endif // ENVCONST

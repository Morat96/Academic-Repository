#include <ros/ros.h>
#include "parameters.h"

using namespace std;

/**
* @brief set DWA Algorithm parameters
*/
void parameters::setDWAparameters() {

	// reconfiguration of DWA parameters
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::IntParameter int_param;
	dynamic_reconfigure::Config conf;
	
	double_param.name = "acc_lim_x";
 	double_param.value = 1.0;
	conf.doubles.push_back(double_param);
	double_param.name = "acc_lim_y";
	double_param.value = 0.0;
	conf.doubles.push_back(double_param);
	double_param.name = "acc_lim_th";
	double_param.value = 2.0;
	conf.doubles.push_back(double_param);
	double_param.name = "max_vel_x";
	double_param.value = 0.2;
	conf.doubles.push_back(double_param);
	double_param.name = "min_vel_x";
	double_param.value = 0.0;
	conf.doubles.push_back(double_param);
	double_param.name = "max_vel_y";
	double_param.value = 0.0;
	conf.doubles.push_back(double_param);
	double_param.name = "min_vel_y";
	double_param.value = 0.0;
	conf.doubles.push_back(double_param);
  	double_param.name = "min_rot_vel";
	double_param.value = 0.4;
	conf.doubles.push_back(double_param);
	double_param.name = "sim_time";
	double_param.value = 4.0;
	conf.doubles.push_back(double_param);
	int_param.name = "vx_samples";
	int_param.value = 15;
	conf.ints.push_back(int_param);
	int_param.name = "vy_samples";
	int_param.value = 1;
	conf.ints.push_back(int_param);
	int_param.name = "vth_samples";
	int_param.value = 30;
	conf.ints.push_back(int_param);
	double_param.name = "path_distance_bias";
	double_param.value = 7.0;
	conf.doubles.push_back(double_param);
	double_param.name = "goal_distance_bias";
	double_param.value = 50.0;
	conf.doubles.push_back(double_param);
	double_param.name = "occdist_scale";
	double_param.value = 0.009;
	conf.doubles.push_back(double_param);
	
	srv_req.config = conf;
	ros::service::call("move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
    ROS_INFO("Re-configuration complete");
}

/**
* @brief update AMCL particles
*/
void parameters::updateAMCLParticles(ros::NodeHandle n) {

	ros::ServiceClient srv = n.serviceClient<std_srvs::Empty>("request_nomotion_update");
    std_srvs::Empty empty;
	
    if (srv.call(empty)) {
    	ROS_INFO("Particles updated");               
    }
    else {
    	ROS_ERROR("Failed to call service");
	}
	sleep(1.0);
}

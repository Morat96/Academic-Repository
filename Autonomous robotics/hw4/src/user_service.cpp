#include "ros/ros.h"
#include "hw4/User.h"

using namespace std;

// User Service
bool add(hw4::User::Request &req, hw4::User::Response &res)
{
	string status = req.arrived;
	ROS_INFO("Robot state: %s", status.c_str());
	ROS_INFO("Number of objects still on the table: %d", req.objOnTable);
	int respGo, respReqObjs;
	ROS_INFO("User Response: 1 -> go, 0 -> stop");
	cin >> respGo;
	res.go = respGo;
	if(respGo == 0) {
		res.req_objs = 0;
		ROS_INFO("The robot has been stopped");
	}
	else {
		ROS_INFO("Number of objects to load in the current run");
		cin >> respReqObjs;
		res.req_objs = respReqObjs;
		ROS_INFO("%d objects lo load in the current run\n", res.req_objs);
	}

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "user_service");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("marrtino/user_service", add);
	ROS_INFO("User Service Active");

	ros::spin();

	return 0;
}





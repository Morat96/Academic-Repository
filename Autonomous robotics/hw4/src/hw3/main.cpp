#include <ros/ros.h>
// Navigator Class
#include "navigation.h"
// external parameters
#include "parameters.h"

using namespace std;

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "navigation");
    ros::NodeHandle n;

	// set DWA parameters
	parameters::setDWAparameters();
    
    // Publisher of cmd_vel for the robot
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("marrtino_base_controller/cmd_vel", 1);
    
    // object of class Navigator
    Navigator *marrtino = new Navigator(n, pub);
    
    // set loop rate
    ros::Rate loop_rate(10);
    
    ros::spinOnce();
    
    while(ros::ok()) {
        
        // publish the message only when there is a subscriber
        if(pub.getNumSubscribers() > 0) {
            // When the robot is at the start position, set the first goal
            if (marrtino -> getPhase().compare("") == 0) {

				// User Service Client 
				ROS_INFO("Request to go sent to the user");

				ros::ServiceClient user_srv = n.serviceClient<hw4::User>("user_service");
                hw4::User set_srv;
                set_srv.request.arrived = "i'm at start position";
				set_srv.request.objOnTable = marrtino -> getNumberObjectsOnTable();
                if (user_srv.call(set_srv)) {
                    if(set_srv.response.go == 1) {
                        ROS_INFO("Go!");
                        marrtino -> setNumberObjectsToLoad(set_srv.response.req_objs);
                        marrtino -> setState(0);
                    }
                    else {
                        ROS_INFO("Stop!");
                        marrtino -> setState(7);
                    }
                }
                else {
                    ROS_ERROR("Failed to call service");
					return 0;
                }

                // first goal: end of the corridor
                Point first_goal(-1.28, 2.98);
                marrtino -> setGoal(first_goal);
                marrtino -> setPhase("main_corridor_navigation");
                // cross the corridor using the Navigation Corridor Algorithm
                marrtino -> setNavigationCorridor(true);
                ROS_INFO("Goal acquired");
            }
            // state 0: set angle (yaw) to robot for the goal pose
            if (marrtino -> getState() == 0) {
                Point goal_yaw = marrtino -> getGoal();
                marrtino -> fix_yaw(goal_yaw);
            }
            // state 1: go straight on until reaching the goal
            else if (marrtino -> getState() == 1) {
                Point goal_straight = marrtino -> getGoal();
                marrtino -> go_straight_ahead(goal_straight);
            }
            // state 2: goal reached
            else if (marrtino -> getState() == 2){
                marrtino -> done();
                marrtino -> setNewGoal();
            }
            // state 3: cross the corridor using the Navigation Corridor Algorithm
            else if (marrtino -> getState() == 3) {
                marrtino -> follow_corridor();
            }
            // state 4: use DWA planner to navigate into Open Space
            else if (marrtino -> getState() == 4) {
                marrtino -> navigateOpenSpace();
            }
            // state 5: go through the free loading station
            else if (marrtino -> getState() == 5) {
                marrtino -> handleLoadPosition();
            }
            // state 6: handle problems about robot stuck
            else if (marrtino -> getState() == 6) {
                marrtino -> handleStuckedRobot();
			}
            // state 7: finished sequence, exit the program
            else if (marrtino -> getState() == 7) {
                break;
            }
            // other non-definied states
            else {
                ROS_INFO("Unknown state");
            }
            
            ros::spinOnce();
            loop_rate.sleep();
            
        } // end if
    } // end while
    
    return 0;
}


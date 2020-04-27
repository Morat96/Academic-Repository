#include <ros/ros.h>
// Navigator Class
#include "navigation.h"

using namespace std;
using namespace move_base_msgs;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//////////////////////// CONSTANTS ////////////////////////

// parameters
const float yaw_precision_ = M_PI / 90; // +/- 2 degrees allowed
const float dist_precision_ = 0.13;

// partial goals
const Point START_POINT(-1.31, 0.22);
const Point FIRST_TURN_START(-1.28, 2.98); // -1.28, 3.17
const Point FIRST_TURN_END(-1.20, 3.2);
const Point GATE_SELECTION(-0.55, 3.14); 
const Point GATE_1_OS(-0.55, 2.67);
const Point GATE_2_TURN(1.18, 3.25);
const Point GATE_2_OS(1.19, 2.34);
const Point LOAD_POINT(-0.482654, 0.56774);
const Point LOAD_POINT_2(0.021878, 0.557094);
const Point CORRIDOR_2_ENTRANCE(0.90, 3.77);
const Point END_CORRIDOR_2_START(-1.21, 3.755);
const Point END_CORRIDOR_2_END(-1.32, 3.70);

// Pose for robot rotating at the end of the main corridor
const Point START_ROTATE_1(-1.26, 0.12);
const Point START_ROTATE_2(-1.31, 0.22);
const Point START_ROTATE_3(-1.31, 0.26);

///////////////////////////////////////////////////////////

/**
 * @brief Class constructor
 */
Navigator::Navigator(ros::NodeHandle n, ros::Publisher pub) {
    
    Navigator::n = n;
    
    // subscribe from LaserScan
    sub_scan = n.subscribe("scan", 1000, &Navigator::callback_scan, this);
    
    // subscribe from Odometry
    sub_odom = n.subscribe("marrtino_base_controller/odom", 1000, &Navigator::callback_odom, this);
    
    // set Publisher
	Navigator::pub = pub;

    // define odom variable
    Navigator::yaw_ = 0;
    
    // laser scan informations
    Navigator::front_distance_ = 0;
    Navigator::left_ = 0;
    Navigator::right_ = 0;
    Navigator::front_right_ = 0;
    Navigator::front_left_ = 0;
    Navigator::left_dist = 0;
    Navigator::right_dist = 0;
    
    // state and phase informations
    Navigator::state_ = 0;
    Navigator::phase_ = "";
    
    // orientation for DWA planner
    Navigator::goal_orientation_x_ = 0;
    Navigator::goal_orientation_y_ = 0;
    Navigator::goal_orientation_z_ = 0;
    Navigator::goal_orientation_w_ = 0;
    
    // set Point variables
	// set the start Point of marrtino
    Navigator::currentPosition_ = Point(-1.31, 0);
    Navigator::goal_ = Point(0, 0);
    Navigator::lastCheckedPosition_ = Point(0, 0);
    
    // utils variables
    Navigator::navigating_corridor = false;
    Navigator::is_loaded_ = false;
    Navigator::is_stucked_ = true;
    Navigator::load_position_ = 1;
    Navigator::numberObjectsToLoad = 0;
	Navigator::numberObjectsOnTable = 7;
    
}

/**
 * @brief callback function of laser scan topic
 * @param msg Laser Scan Message
 */
void Navigator::callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    
    // find mean front distance (used to detect obstacles in gate 2)
    int i;
    float front_distance = 0;
    
    for (i = 0; i < 30; i++) {
        front_distance += msg -> ranges[186 + i];
    }
    
    front_distance_ = front_distance / i;
    
    // find mean left and right distance (used to navigate in corridors)
    float left_distance = 0;
    float right_distance = 0;
    int l;
    for(l = 0; l < 30; l++) {
        right_distance += msg -> ranges[86 + l];
        left_distance += msg -> ranges[286 + l];
    }
    
    right_ = right_distance / l;
    left_ = left_distance / l;
    
    int m;
    // other directions to check, front right and front left
    float front_right_distance = 0;
    float front_left_distance = 0;
    for(m = 0; m < 30; m++) {
        front_right_distance += msg -> ranges[136 + l];
        front_left_distance += msg -> ranges[236 + l];
    }
    
    front_right_ = front_right_distance / m;
    front_left_ = front_left_distance / m;

    // laser informations for the Corridor Algorithm
    for(int i=0;i<20;i++) {
        right_dist += msg -> ranges[139+i];
        left_dist += msg -> ranges[239+i];
    }
    
    right_dist = right_dist/20;
    left_dist = left_dist/20;
    
}

/**
 * @brief callback function of Odom topic
 * @param msg Odom Message
 */
void Navigator::callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    
    // update current position
    currentPosition_ = Point(msg -> pose.pose.position.x, msg -> pose.pose.position.y);

    // current orientation
    tf::Quaternion quaternion(
                              msg -> pose.pose.orientation.x,
                              msg -> pose.pose.orientation.y,
                              msg -> pose.pose.orientation.z,
                              msg -> pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    // get yaw
    yaw_ = yaw;
    
}

/**
 * @brief function that change current state
 */
void Navigator::change_state(int state) {
    state_ = state;
    ROS_INFO("State changed to %d", state);
}

/**
 * @brief follow corridor Algorithm
 */
float Navigator::follow_corridor() {
    
    float err_pos = sqrt(pow(goal_.y - currentPosition_.y, 2) + pow(goal_.x - currentPosition_.x, 2));
    if (abs(left_dist - right_dist) > 0.1) {
        change_state(1);
    }
    
    geometry_msgs::Twist msg;
    geometry_msgs::Point point;
    float alpha = 0;
    
    // set angle: 45°
    float param = 45.0;
    float result_sin = sin (param*M_PI/180);
    float result_cos = cos (param*M_PI/180);
    
    // compute the width between the two walls using previous distances and
    // the angle compute width left and right of the corridor width left
    float width_sx = left_dist * result_sin;
    // width right
    float width_dx = right_dist * result_sin;
    // compute distance from center of the robot and the position forward
    float forward_distance = left_dist * result_cos;
    // compute the width
    float width = width_sx + width_dx;
    // compute the center of the width
    width = width/2;
    
	// follow the main corridor 
    if (phase_.compare("main_corridor_navigation") == 0) {
        
        // compute the extreme left point of the width
        float extr_point_left = currentPosition_.x - width_sx;
        // compute the final x position
        float final_x = extr_point_left + width;
        
        // update Point goal
        point.x = final_x;
        point.y = currentPosition_.y + forward_distance;
        
        // compute the distance between the actual x and final x
        float distance = sqrt(pow(currentPosition_.x - point.x, 2.0) + pow(currentPosition_.y - point.y, 2.0));
        // compute the goal angle
        float cos_angle = forward_distance/distance;
        float final_angle = acos(cos_angle);
        
        if(point.x > currentPosition_.x) alpha = - final_angle;
        else alpha = final_angle;
       
        msg.linear.x = 0.2;
        msg.angular.z = alpha;
        
    }

	// follow the second corridor
    if (phase_.compare("navigate_corridor_2") == 0) {
        
        // compute the extreme left point of the width
        float extr_point_left = currentPosition_.y - width_sx;
        // compute the final x position
        float final_y = extr_point_left + width;
        
        // update Point goal
        point.y = final_y;
        point.x = currentPosition_.x - forward_distance;
        
        // compute the distance between the actual x and final x
        float distance = sqrt(pow(currentPosition_.x-point.x, 2.0) + pow(currentPosition_.y-point.y, 2.0));
        // compute the goal angle
        float cos_angle = forward_distance/distance;
        float final_angle = acos(cos_angle);
        
        if(point.y > currentPosition_.y) alpha = - final_angle;
        else alpha = + final_angle;
       
        msg.linear.x = 0.2;
        msg.angular.z = alpha;
        
    }

	// follow the main corridor (reverse)
    if (phase_.compare("back_to_start") == 0) {
        
        // compute the extreme left point of the width
        float extr_point_left = currentPosition_.x + width_sx;
        // compute the final x position
        float final_x = extr_point_left - width;
        
        // update Point goal
        point.x = final_x;
        point.y = currentPosition_.y - forward_distance;
        
        // compute the distance between the actual x and final x
        float distance = sqrt(pow(currentPosition_.x - point.x, 2.0) + pow(currentPosition_.y - point.y, 2.0));
        // compute the goal angle
        float cos_angle = forward_distance/distance;
        float final_angle = acos(cos_angle);
        
        if(point.x > currentPosition_.x) alpha = + final_angle;
        else alpha = - final_angle;
        
        msg.linear.x = 0.2;
        msg.angular.z = alpha;
    }
    
    pub.publish(msg);
    
}

int mapAngleToLaser(float angle) {
	float epsilon = M_PI / 8;
	if (angle < M_PI / 2 + epsilon && angle > M_PI / 2 - epsilon) {
		// goal is on the right
		return 0;
	} else if (angle < M_PI / 4 + epsilon && angle > M_PI / 4 - epsilon) {
		// goal is on the front right
		return 1;
	} else if (abs(angle) < epsilon) {
		// goal is forward
		return 2;
	} else if (angle < -M_PI / 4 + epsilon && angle > -M_PI / 4 - epsilon) {
		// goal is on the front left
		return 3;
	} else if (angle < -M_PI / 2 + epsilon && angle > -M_PI / 2 - epsilon) {
		// goal is on the left
		return 4;
	} else {
		ROS_INFO("Can't map an angle of %f", angle);
		return 5;
	}
}

/**
 * @brief get angle between current position and target position
 * @param targetPoint
 */
void Navigator::fix_yaw(Point targetPoint) {
    
    float desired_yaw = atan2(targetPoint.y - currentPosition_.y, targetPoint.x - currentPosition_.x);
    if(yaw_> 2.61 || yaw_< -2.61) {
        if(desired_yaw < 0){
            desired_yaw += 6.28;
        }
        if(yaw_ < 0){
            yaw_ += 6.28;
        }
    }
    
    float err_yaw = desired_yaw - yaw_;
    
    
    geometry_msgs::Twist msg;
    if (abs(err_yaw) > yaw_precision_) {
        if (err_yaw > 0) msg.angular.z = 0.2;
        else msg.angular.z = -0.2;
    }
    
    pub.publish(msg);
    
    // state change conditions
    if (abs(err_yaw) <= yaw_precision_) {
       
        change_state(1);
    }
}

/**
 * @brief go straight on until reaching the goal
 * @param targetPoint
 */
void Navigator::go_straight_ahead(Point targetPoint) {
    float desired_yaw = atan2(targetPoint.y - currentPosition_.y, targetPoint.x - currentPosition_.x);
    float err_yaw = desired_yaw - yaw_;
    float err_pos = sqrt(pow(targetPoint.y - currentPosition_.y, 2) + pow(targetPoint.x - currentPosition_.x, 2));
  
    if (err_pos > dist_precision_) {
        geometry_msgs::Twist msg;
        
        if (err_pos > 0.4) {
            msg.linear.x = 0.2;
        } else {
            // IF next to the goal, publish a smaller linear speed for better refining
            msg.linear.x = 0.1;
        }
        
        pub.publish(msg);
        
    } else {
       
        change_state(2);
    }
    
    // state change conditions
    if (navigating_corridor && err_pos > 0.3 && (abs(left_dist - right_dist) < 0.1)) {
        // float yaw_corridor = get_yaw_corridor();
        change_state(3);
    } else {
        if (abs(err_yaw) > yaw_precision_) {
            
            change_state(0);
        }
        
    }
}

/**
 * @brief turn off the engines
 */
void Navigator::done() {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    pub.publish(msg);
}

/**
 * @brief set a new goal pose
 */
void Navigator::setNewGoal() {
    ROS_INFO("Searching for a goal...");
    if (phase_.compare("main_corridor_navigation") == 0) {
        ROS_INFO("Complete turn");
        phase_ = "complete_turn_1";
        goal_ = FIRST_TURN_END;
        navigating_corridor = false;
        change_state(0);
    } else if (phase_.compare("complete_turn_1") == 0) {
        ROS_INFO("Navigate to gate selection");
        phase_ = "to_gate_selection";
        goal_ = GATE_SELECTION;
        navigating_corridor = false;
        change_state(0);
    }else if (phase_.compare("to_gate_selection") == 0){
        // check if gate 2 is open and set the goal consequently
        ROS_INFO("front_distance_ >>> %f", front_distance_);
        if (front_distance_ > 1.3) {
            ROS_INFO("Navigate through gate 2");
            phase_ = "through_gate_2";
            goal_ = GATE_2_TURN;
            navigating_corridor = false;
            change_state(0);
        } else {
            ROS_INFO("Enter open space through gate 1");
            phase_ = "open_space_entrance";
            goal_ = GATE_1_OS;
            navigating_corridor = false;
            change_state(0);
        }
    } else if (phase_.compare("through_gate_2") == 0){
        ROS_INFO("Enter open space");
        phase_ = "open_space_entrance";
        goal_ = GATE_2_OS;
        navigating_corridor = false;
        change_state(0);
    } else if (phase_.compare("open_space_entrance") == 0) {
        ROS_INFO("Navigate in open space to the desired goal");
        phase_ = "open_space_navigation";
        goal_ = LOAD_POINT;
        goal_orientation_z_ = -0.7071068;
        goal_orientation_w_ = 0.7071068;
        navigating_corridor = false;
        change_state(4);
    } else if (phase_.compare("open_space_navigation") == 0) {
        ROS_INFO("Exit from open space");
        phase_ = "open_space_exit";
        goal_ = GATE_2_OS;
        goal_orientation_z_ = -0.7071068;
        goal_orientation_w_ = -0.7071068;
        navigating_corridor = false;
        change_state(4);
    } else if (phase_.compare("open_space_exit") == 0) {
        ROS_INFO("Navigate to gate 2");
        phase_ = "exit_from_gate_2";
        goal_ = GATE_2_TURN;
        navigating_corridor = false;
        change_state(0);
    } else if (phase_.compare("exit_from_gate_2") == 0) {
        ROS_INFO("Navigate to corridor 2 entrance");
        phase_ = "to_corridor_2_entrance";
        goal_ = CORRIDOR_2_ENTRANCE;
        navigating_corridor = false;
        change_state(0);
    } else if (phase_.compare("to_corridor_2_entrance") == 0) {
        ROS_INFO("Navigate to corridor 2 end");
        phase_ = "navigate_corridor_2";
        goal_ = END_CORRIDOR_2_START;
        navigating_corridor = true;
        change_state(0);
    } else if (phase_.compare("navigate_corridor_2") == 0) {
        ROS_INFO("Intermediate pose before turn to start");
        phase_ = "navigate_corridor_2_to_start";
        goal_ = END_CORRIDOR_2_END;
        navigating_corridor = false;
        change_state(0);
    } else if (phase_.compare("navigate_corridor_2_to_start") == 0) {
        ROS_INFO("Navigate back to start");
        phase_ = "back_to_start";
        goal_ = START_POINT;
        navigating_corridor = true;
        change_state(0);
    } else if (phase_.compare("back_to_start") == 0) {
        ROS_INFO("Rotating the robot");
        phase_ = "start_rotate_1";
		goal_ = START_ROTATE_1;
        navigating_corridor = false;
        change_state(0);
    } else if (phase_.compare("start_rotate_1") == 0) {
        phase_ = "start_rotate_2";
		goal_ = START_ROTATE_2;
        navigating_corridor = false;
		change_state(0);
    } else if (phase_.compare("start_rotate_2") == 0) {
        ROS_INFO("Finished rotation");
        phase_ = "re-start";
		goal_ = START_ROTATE_3;
        navigating_corridor = false;

		// User Service Client
        ROS_INFO("Request to go sent to the user");
		ros::ServiceClient user_srv = n.serviceClient<hw4::User>("user_service");
        hw4::User set_srv;
        set_srv.request.arrived = "i'm at start position";
		set_srv.request.objOnTable = numberObjectsOnTable;
        if (user_srv.call(set_srv)) {
        	if(set_srv.response.go == 1) {
        		ROS_INFO("Go!");
        		numberObjectsToLoad = set_srv.response.req_objs;
        		change_state(0);
       		}
       		else {
       			ROS_INFO("Stop!");
     			change_state(7);
      		}
        }
        else {
       		ROS_ERROR("Failed to call service");
			change_state(7);
       	}
    } else if (phase_.compare("re-start") == 0) {
		ROS_INFO("Start a new run");
        phase_ = "main_corridor_navigation";
		goal_ = FIRST_TURN_START;
        navigating_corridor = true;
		change_state(0); 
	}
}

/**
 * @brief navigate into Open Space using DWA algorithm
 */
void Navigator::navigateOpenSpace() {
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;
    
    //we'll send a goal to the robot
    goal.target_pose.header.frame_id = "marrtino_map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    // pose
    goal.target_pose.pose.position.x = goal_.x;
    goal.target_pose.pose.position.y = goal_.y;
    
    // orientation
    // goal.target_pose.pose.orientation.x = goal_orientation_x_;
    // goal.target_pose.pose.orientation.y = goal_orientation_y_;
    goal.target_pose.pose.orientation.z = goal_orientation_z_;
    goal.target_pose.pose.orientation.w = goal_orientation_w_;
    
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult(ros::Duration(5.0));
    
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal reached");
        if (phase_.compare("open_space_navigation") == 0) {
            if (load_position_ == 1 || load_position_ == 2) {
				if (load_position_ == 1) goal_ = Point(-0.482654, 0.495774);
				if (load_position_ == 2) goal_ = Point(0.021878, 0.527094);
                change_state(5);
            } else {
                // ripristine goal
                is_stucked_ = false;
                load_position_ = 1;
                goal_ = LOAD_POINT;
            }
        } else {
            change_state(2);
        }
    } else if (ac.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
        ROS_INFO("Client is in active status");
        // if lastCheckedPosition is still to (0,0), it means it's the first check, so save it
        change_state(6);
    } else { // generic status, not handled
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Robot stucked in state %s", state.toString().c_str());
    }
}

/**
 * @brief go through the free loading station
 */
void Navigator::handleLoadPosition() {
    // check yaw from current goal
    tf::Quaternion quaternion(0, 0, goal_orientation_z_, goal_orientation_w_);
    double desired_roll, desired_pitch, desired_yaw;
    tf::Matrix3x3(quaternion).getRPY(desired_roll, desired_pitch, desired_yaw);
    
    float err_yaw = desired_yaw - yaw_;
    float err_pos = sqrt(pow(goal_.y - currentPosition_.y, 2) + pow(goal_.x - currentPosition_.x, 2));
    
    ROS_WARN("err_yaw >>> %f", err_yaw);
    ROS_WARN("err_pos >>> %f", err_pos);
    
    geometry_msgs::Twist msg;
    if (!is_loaded_) {
        // fix yaw before
        if (abs(err_yaw) > yaw_precision_) {
            if (err_yaw > 0) msg.angular.z = 0.1;
            else msg.angular.z = -0.1;
            pub.publish(msg);
        } else {
            // if yaw is already correct, fix distance
            if (err_pos > dist_precision_ - 0.06) {
                msg.linear.x = 0.1;
                pub.publish(msg);
            } else {
                ROS_INFO("Reached load position");
				done();
				// start the sequence of the manipulator robot by sending a service specifying the number of the
		        // loading station and the number of objects to be downloaded to the marrtino.
		        // check first in which loading station we are.
		        ros::ServiceClient load_srv = n.serviceClient<hw4::srv1>("/ur5/hw4");
		        hw4::srv1 srv;
		        if(load_position_ == 1) {
		            srv.request.station = "load1";
		            srv.request.objects = numberObjectsToLoad;
		            if (load_srv.call(srv)) {
		                ROS_INFO("Number of objects still on the table: %ld", (long int)srv.response.objOnTable);
						numberObjectsOnTable = srv.response.objOnTable;
		            }
		            else {
		                ROS_ERROR("Failed to call service");
		            }
		        }
		        if(load_position_ == 2) {
		            srv.request.station = "load2";
		            srv.request.objects = numberObjectsToLoad;
		            if (load_srv.call(srv)) {
		                ROS_INFO("Number of objects still on the table: %ld", (long int)srv.response.objOnTable);
						numberObjectsOnTable = srv.response.objOnTable;
		            }
		            else {
		                ROS_ERROR("Failed to call service");
		            }
		        }
				sleep(4.0);
                // now call service to be loaded (which will set is_loaded_ to true once he has finished)
                is_loaded_ = true;
                // update goal
                goal_.y += 0.8;
                ROS_INFO("Goal is updated to >>> ( %f , %f )", goal_.x, goal_.y);
            }
        }
    } else {
        // robot is loaded, drift away from load position
        if (err_pos > dist_precision_) {
            msg.linear.x = -0.1;
            pub.publish(msg);
        } else {
            ROS_INFO("Drifted from load position");
            change_state(2);
        }
        
    }
}

bool Navigator::checkIfFreePath(float distanceFromGoal, int mappedAngle) {
	if (mappedAngle == 0) return distanceFromGoal > right_;
	else if (mappedAngle == 1) return distanceFromGoal > front_right_;
	else if (mappedAngle == 2) return distanceFromGoal > front_distance_;
	else if (mappedAngle == 3) return distanceFromGoal > front_left_;
	else if (mappedAngle == 4) return distanceFromGoal > left_;
	else return false;
}

/**
 * @brief handle problems about robot stuck
 */
void Navigator::handleStuckedRobot() {
    
    if (lastCheckedPosition_.x == 0 && lastCheckedPosition_.y == 0) {
        lastCheckedPosition_ = currentPosition_;
        change_state(4);
    } else {
        
        // evaluate done distance from previous check
        float distance = sqrt(
                              pow(lastCheckedPosition_.y - currentPosition_.y, 2) +
                              pow(lastCheckedPosition_.x - currentPosition_.x, 2)
                              );
        ROS_INFO("Distance done from previous check >>> %f", distance);
        if (distance < 0.2) {    // robot has not/slightly moved in the last 5 seconds
            if (!is_loaded_) { // robot is going to load position
                // try change goal
                if (load_position_ == 1) {
                    ROS_INFO("Can't get to load position 1, try with load position 2");
                    load_position_ = 2;
                    goal_ = LOAD_POINT_2;
                } else if (load_position_ == 2) {
                    ROS_INFO("Can't get to load position 2, try a random goal");
                    // manual recovery
                    load_position_ = 3; // generic goal
                    goal_ = getRandomPoint();
                } else if (load_position_ == 3) {
                    // new random goal
                    goal_ = getRandomPoint();
                }
                // back to navigation stack
    		change_state(4);
            } else {
            	float currDistanceFromGoal = sqrt(
            		pow(currentPosition_.y - goal_.y, 2) +
                        pow(currentPosition_.x - goal_.x, 2)
            	);
            	float desired_yaw = atan2(goal_.y - currentPosition_.y, goal_.x - currentPosition_.x);
            	int laserAngleCase = mapAngleToLaser(desired_yaw);
            	if (checkIfFreePath(currDistanceFromGoal, laserAngleCase)) {
            		change_state(0);
            	} else {
            		change_state(4);
            	}
            }
        } else {
        	change_state(4);
        }
        lastCheckedPosition_ = currentPosition_;
    }
}

/**
 * @brief function that get a random Point
 */
Point Navigator::getRandomPoint() {
    // start from current position and determine ranges in which search for a random point
    float min_x = currentPosition_.x - 0.3;
    if (min_x < -1.0) min_x = -1.0;
    float min_y = currentPosition_.y - 0.7;
    if (min_y < 0.5) min_y = 0.5;
    
    float max_x = currentPosition_.x + 0.7;
    if (max_x > 0.5) max_x = 0.5;
    float max_y = currentPosition_.y + 0.3;
    if (max_y > 2.7) max_y = 2.7;
    
    float random_x = min_x + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(max_x - min_x)));
    float random_y = min_y + static_cast <float> (rand())/(static_cast <float> (RAND_MAX/(max_y - min_y)));
    
    Point point(random_x, random_y);
    ROS_INFO("New goal >>> (%f,%f)", point.x, point.y);
    return point;
}

/**
 * @brief set current state
 */
void Navigator::setState(int state) {
    state_ = state;
}

/**
 * @brief get current state
 */
int Navigator::getState() {
    return state_;
}

/**
 * @brief set a new phase
 */
void Navigator::setPhase(string phase) {
    Navigator::phase_ = phase;
}

/**
 * @brief get current phase
 */
string Navigator::getPhase() {
    return phase_;
}

/**
 * @brief set a new goal
 */
void Navigator::setGoal(Point goal) {
    Navigator::goal_ = goal;
}

/**
 * @brief get the current goal
 */
Point Navigator::getGoal() {
    return goal_;
}

/**
 * @brief set if to use the Navigation Corridor Algorithm
 */
void Navigator::setNavigationCorridor(bool value) {
    Navigator::navigating_corridor = value;
}

/**
 * @brief set the number of objects to load on marrtino
 */
void Navigator::setNumberObjectsToLoad(int value) {
    Navigator::numberObjectsToLoad = value;
}

/**
* @brief get the number of objects still on the table
*/
int Navigator::getNumberObjectsOnTable() {
	return numberObjectsOnTable;
}

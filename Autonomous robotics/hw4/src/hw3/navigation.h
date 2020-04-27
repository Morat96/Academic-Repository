#ifndef Navigator_
#define Navigator_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <math.h>
#include <cmath>
#include "hw4/srv1.h"
#include "hw4/User.h"

using namespace std;

// struct to define a pair of coordinates
struct Point {
    
    float x, y;
    // constructor
    Point(float a, float b) {
        this -> x = a;
        this -> y = b;
    }
    // empty constructor
    Point() { }
};

class Navigator
{
public:
    
    /**
     * @brief Class constructor
     */
    Navigator(ros::NodeHandle n, ros::Publisher pub);
    
    // ******************** CALLBACK FUNCTIONS ********************
    
    /**
     * @brief callback function of laser scan topic
     * @param msg Laser Scan Message
     */
    void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    /**
     * @brief callback function of Odom topic
     * @param msg Odom Message
     */
    void callback_odom(const nav_msgs::Odometry::ConstPtr& msg);
    
    // ************************************************************
    
    // *************** FUNCTIONS FOR MOVING THE ROBOT ***************
    
    /**
     * @brief follow corridor Algorithm
     */
    float follow_corridor();
    
    /**
     * @brief get desired yaw between current position and target position
     * @param targetPoint
     */
    void fix_yaw(Point targetPoint);
    
    /**
     * @brief go straight on until reaching the goal
     * @param targetPoint
     */
    void go_straight_ahead(Point targetPoint);
    
    /**
     * @brief turn off the engines
     */
    void done();
    
    /**
     * @brief set a new goal pose
     */
    void setNewGoal();
    
    /**
     * @brief navigate into Open Space using DWA algorithm
     */
    void navigateOpenSpace();
    
    /**
     * @brief go through the free loading station
     */
    void handleLoadPosition();
    
    /**
     * @brief handle problems about robot stuck
     */
    void handleStuckedRobot();
    
    // ************************************************************
    
    // ******************* SET AND GET FUNCTIONS ******************
    
    /**
     * @brief set current state
     */
    void setState(int state);

    /**
     * @brief get current state
     */
    int getState();
    
    /**
     * @brief set a new phase
     */
    void setPhase(string phase);
    
    /**
     * @brief get current phase
     */
    string getPhase();
    
    /**
     * @brief set a new goal
     */
    void setGoal(Point goal);
    
    /**
     * @brief get the current goal
     */
    Point getGoal();
    
    /**
     * @brief set if to use the Navigation Corridor Algorithm
     */
    void setNavigationCorridor(bool value);

    /**
     * @brief set the number of objects to load on marrtino
     */
    void setNumberObjectsToLoad(int value);

	 /**
     * @brief get the number of objects still on the table
     */
	int getNumberObjectsOnTable();
    
    // ************************************************************
    
    // ********************** OTHER FUNCTIONS *********************
    
    /**
     * @brief function that changes current state
     */
    void change_state(int state);
    
    /**
     * @brief checks if there exist an obstacle-free path to the goal
     */
    bool checkIfFreePath(float distanceFromGoal, int mappedAngle);
    
    /**
     * @brief function that gets a random Point
     */
	Point getRandomPoint();
    
    // ************************************************************

private:
    
    // ROS variables
    ros::NodeHandle n;
    ros::Subscriber sub_scan;
    ros::Subscriber sub_odom;
    ros::Publisher pub;
    
    // Pose variables
    Point currentPosition_;
    Point goal_;
    Point lastCheckedPosition_;
    
    // utils variables
    bool navigating_corridor;
    bool is_loaded_;
    bool is_stucked_;
    int load_position_;
    int numberObjectsToLoad;
	int numberObjectsOnTable;
    
    // odom variables
    float current_x_;
    float current_y_;
    float yaw_;
    
    // laser scan informations
    float front_distance_;
    float left_;
    float right_;
    float front_right_;
    float front_left_;
    float width_sx;
    float width_dx;
    float forward_distance;
    float width;
    float left_dist;
    float right_dist;
    
    // state and phase informations
    int state_;
    string phase_;
    
    // orientation for DWA planner
    float goal_orientation_x_;
    float goal_orientation_y_;
    float goal_orientation_z_;
    float goal_orientation_w_;
    
};

#endif /* Navigator_ */

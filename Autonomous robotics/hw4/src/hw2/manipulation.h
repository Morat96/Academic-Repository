#ifndef Manipulation_
#define Manipulation_

#include <ros/ros.h>
#include <iostream>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <std_msgs/Header.h>
#include "geometry_msgs/PoseStamped.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <cstdlib>
#include "robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h"
#include <ur_msgs/SetIO.h>
#include "std_msgs/Float32MultiArray.h"
#include "hw4/msg1.h"
#include "hw4/srv1.h"
#include "hw4/Attach.h"
#include <cmath>
#include <math.h>

using namespace std;

class Manipulation
{
public:
    
    /**
    * @brief Class constructor
    */
    Manipulation(moveit::planning_interface::MoveGroupInterface *move_group,
                 moveit::planning_interface::PlanningSceneInterface *planning_scene_interface,
                 const robot_state::JointModelGroup* joint_model_group,
                 vector<apriltag_ros::AprilTagDetection> collisionObjects,
                 vector<apriltag_ros::AprilTagDetection> requestedObjects,
                 int simulation, int gripper_enable);
    
    // *************** FUNCTIONS FOR MOVE THE ROBOT ***************
    
    /**
     * @brief Pose used for free the table from the arm
     */
    void goToExternalPosition();
    
    /**
     * @brief Pose for positioning the arm over the table
     */
    void goToMainPosition();
    
    /**
     * @brief Go over the object i
     * @param i Index of the requested object
     */
    void goToTargetPosition(int i);
    
    /**
     * @brief Approach the object by going down
     * @param meters Offset distance
     */
    double goCartesianDown(double meters);
    
    /**
     * @brief Approach the object by going down (for prism only in real)
     * @param meters Offset distance
     */
    double goCartesianDownPrism(double meters);

	/**
     * @brief place the object on the basket of the mobile robot
     * @param meters Offset distance
     */
	double goCartesianDownBasket(string load, double meters);
    
    /**
     * @brief Pull up the object
     * @param meters Offset distance
     */
    void goCartesianUp(double meters);
    
    /**
     * @brief Positions to go to the final pose
     */
    void goToIntermiatePosesOnwards();
    
    /**
     * @brief Positions to return from the final pose
     * @param i Is the index of object to detach and remove from collision avoidance
     */
    void goToIntermiatePosesBackwards(int i);
    
    /**
     * @brief Go to final pose based on the chosen gate
     * @param load Loading station
     * @param i Index of the object requested by the user
     */
    void goToFinalPose(string load, int i);
    
    /**
     * @brief Recovery function: if the grasping fail, try again
     * @param i Index of the object requested by the user
     */
    bool objectRecovery(int i);
    
    // ************************************************************
    
    // **************** FUNCTIONS FOR OBTAIN POSES ****************
    
    /**
     * @brief Objects on the table
     */
    void obtainCollisionObjects();
    
    /**
     * @brief Objects requested by the user
     */
    void obtainRequestedObjects();
    
    // ************************************************************
    
    // **************** FUNCTIONS FOR ATTACH/DETACH ***************
    
    /**
     * @brief Function for Gripper opening/closing
     * @param pub Publisher for Gripper control
     * @param str Message for Gripper control
     */
    void openGripper(ros::Publisher& pub, robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput& str);
    void closeGripper(ros::Publisher& pub, robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput& str);
    
    /**
     * @brief Function for Magnet activation/deactivation
     * @param srv Service for Magnet control
     * @param set_io Message for Magnet control
     * @param operation Type of operation
     */
    void magnetControl(ros::ServiceClient& srv, ur_msgs::SetIO& set_io, std::string operation);

    /**
     * @brief Function for Link attach/detach
     * @param srv Service for Link attacher
     * @param i Index of the object requested by the user
     */
	void linkAttach(ros::ServiceClient& srv, int i);
    
    // ************************************************************

    // ************ FUNCTIONS FOR COLLISION AVOIDANCE *************
    
    /**
     * @brief Function for collision avoidance
     */
    void applyCollisionObjects();

    /**
     * @brief Attach object collision to arm
     * @param i Index of the object requested by the user
     * @param offset Distance in z to re-add object collision
     */
	void attachObjectCollision(int i, double offset);
    
    // ************************************************************
    
    // ******************** UTILITY FUNCTIONS *********************
    
    /**
     * @brief Clean the world from the collision objects
     */
	void cleanWorld();
    
    /**
     * @brief Get the number of requested objects
     */
    int getNumberRequestedObjects();
    
    /**
     * set requested objects.
     */
    void setRequestedObjects(vector<apriltag_ros::AprilTagDetection> obj);
    
    /**
     * Get requested objects.
     * @return objects
     */
    vector<apriltag_ros::AprilTagDetection> getRequestedObjects();
    
    /**
     * @brief Obtain the object id
     * @param i Index of the object requested by the user
     */
    int obtainIdObject(int i);
    
    // ************************************************************
    
private:
    
    // environment
    bool simulation;
    int gripper_enable;
    
    // moveit variables
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;
    const robot_state::JointModelGroup* joint_model_group;

    // vectors for poses
    vector<apriltag_ros::AprilTagDetection> collisionObjects;
    vector<apriltag_ros::AprilTagDetection> requestedObjects;
    
    // joint values
    std::vector<double> joint_group_positions;
	
    // variable useful for visualize the success of the planning  
    bool success;
    
    // retrive trasformation from reference system /camera to /world
    geometry_msgs::TransformStamped cameratobase;
    // retrive trasformation from reference system /world to /eef_link
    geometry_msgs::TransformStamped worldtoeef;
    // retrive trasformation from reference system /eef_link to /world
    geometry_msgs::TransformStamped eeftoworld;

};

#endif /* Manipulation_ */

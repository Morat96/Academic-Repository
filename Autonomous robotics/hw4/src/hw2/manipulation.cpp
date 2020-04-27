#include <ros/ros.h>
#include "manipulation.h"

using namespace std;

// definition of utils functions
double computeSlopeHeight(double current_x);
bool isPrism(int id);
string obtainObjectType(int id);

// ****************************************** CONSTANTS ****************************************** //

// objects' heights: simulation
const double cube_z_sim = 0.95;
const double cyl_z_sim = 1.08;
const double prism_z_sim = 0.895;

// objects' heights: real
const double cube_z_real = 0.91;
const double cyl_z_real = 1.071;
const double prism_z_real = 0.908;

// define name of models of links of Gazebo objects
const string model_types[16] = {"cube1", "cube2", "cube3", "cube4", "Hexagon0",
	"Hexagon1", "Triangle0", "Triangle1", "Triangle2", "blue_cube_1",
	"blue_cube_2", "blue_cube_3", "blue_cube_4", "red_triangle_1", "red_triangle_2", "red_triangle_3"};

const string link_types[16] = {"cube1_link", "cube2_link", "cube3_link", "cube4_link", "Hexagon0_link",
	"Hexagon1_link", "Triangle0_link", "Triangle1_link", "Triangle2_link", "blue_cube_1_link",
	"blue_cube_2_link", "blue_cube_3_link", "blue_cube_4_link", "red_triangle_1_link", "red_triangle_2_link", "red_triangle_3_link"};

// ************************************************************************************************  //

/**
 * Class constructor.
 */
Manipulation::Manipulation(moveit::planning_interface::MoveGroupInterface *move_group,
			   moveit::planning_interface::PlanningSceneInterface *planning_scene_interface,
                           const robot_state::JointModelGroup* joint_model_group,
                           vector<apriltag_ros::AprilTagDetection> collisionObjects,
                           vector<apriltag_ros::AprilTagDetection> requestedObjects,
                           int simulation, int gripper_enable) {

    Manipulation::move_group = move_group;
    Manipulation::planning_scene_interface = planning_scene_interface;
    Manipulation::joint_model_group = joint_model_group;
    Manipulation::collisionObjects = collisionObjects;
    Manipulation::requestedObjects = requestedObjects;
    Manipulation::simulation = simulation;
    Manipulation::gripper_enable = gripper_enable;
    
    Manipulation::current_state = move_group->getCurrentState();
    
    // subscribe from tf and obtain TF from /camera to /world in order to give to ur5 poses from its referement system
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2_listener(tfBuffer);
    
    Manipulation::cameratobase = tfBuffer.lookupTransform("world", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(1.0));
    
    // obtain transformation from and to /ee_link and /world
    Manipulation::worldtoeef = tfBuffer.lookupTransform("ee_link", "world", ros::Time(0), ros::Duration(1.0));
    Manipulation::eeftoworld = tfBuffer.lookupTransform("world", "ee_link", ros::Time(0), ros::Duration(1.0));
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////// FUNCTIONS FOR MOVE THE ROBOT //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * External position. Pose used for free the table from the arm, allowing correctly to camera to see the objects on the table.
 */
void Manipulation::goToExternalPosition() {
    
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group->setStartStateToCurrentState();
    
    // set joints
    joint_group_positions[0] = 1.69227;
    joint_group_positions[1] = -0.765606;
    joint_group_positions[2] = -1.8354;
    joint_group_positions[3] = -2.15177;
    joint_group_positions[4] = 1.55672;
    joint_group_positions[5] = 0.122668;

    move_group->setJointValueTarget(joint_group_positions);
    
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("hw2", "EXTERNAL POSE %s", success ? "" : "FAILED");
    
    // compute the plan and move the robot
    move_group->move();
}

/**
 * Main position. Pose for positioning the arm over the table.
 */
void Manipulation::goToMainPosition() {
    
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group->setStartStateToCurrentState();

    // set joints
    joint_group_positions[0] = 1.6659278869628906;
    joint_group_positions[1] = -1.86481219926943;
    joint_group_positions[2] = -1.0905783812152308;
    joint_group_positions[3] = -1.6998422781573694;
    joint_group_positions[4] = 1.543563723564148;
    joint_group_positions[5] = 0.008275856263935566;
    
    move_group->setJointValueTarget(joint_group_positions);
    
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("hw2", "MAIN POSE %s", success ? "" : "FAILED");
    
    // compute the plan and move the robot
    move_group->move();
    
}

/**
 * Target position. Go over the object i.
 * @param i Index of the requested object.
 */
void Manipulation::goToTargetPosition(int i) {

    // target pose
    geometry_msgs::Pose target_pose;
    
    //obtain the pose of the object
    geometry_msgs::Pose obj_pose;
    
    obj_pose.position = requestedObjects[i].pose.pose.pose.position;
    obj_pose.orientation = requestedObjects[i].pose.pose.pose.orientation;
    
    // set the position of the object with the reference system of world
    target_pose.position.x = obj_pose.position.x;
    target_pose.position.y = obj_pose.position.y;
    
	// obtain a pose that have EEF facing down to grasp
	tf::Quaternion init_quat(obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w);
		
	double roll, pitch, yaw;
	tf::Quaternion final_quat(0,0,0,1);
		
	// get the RPY of the pose
	tf::Matrix3x3(init_quat).getRPY(roll, pitch, yaw);

	int id = requestedObjects[i].id[0];
    if(gripper_enable) {
        if(isPrism(id)) {
            roll = 0;
            pitch = M_PI_2;
            yaw += M_PI_2;
        }
        else {
            roll = 0;
            pitch = M_PI_2;
        }
        // set the position of eef over the object of 8 cm
        target_pose.position.z = obj_pose.position.z + 0.30;
    }
    else {
        if(isPrism(id)) {
            // compute a rotation of the pitch, roll and yaw in order to correctly set the eef orientation for Prisms
            roll = 2.86;
            pitch = 0.8;
            yaw -= M_PI_2;
            // set the position of eef over the object of 8 cm
            target_pose.position.z = obj_pose.position.z + 0.08;
        }
        else {
            // compute a rotation of the pitch and yaw of 90 degrees for other objects
            roll = 0;
            pitch = M_PI_2;
            // set the position of eef over the object of 20 cm
            target_pose.position.z = obj_pose.position.z + 0.20;
        }
    }
    
    final_quat.setRPY(roll, pitch, yaw);
    
    // set the orientation
    target_pose.orientation.w = final_quat.w();
    target_pose.orientation.x = final_quat.x();
    target_pose.orientation.y = final_quat.y();
    target_pose.orientation.z = final_quat.z();
    
    // slows down the movement of the robot
	move_group->setMaxVelocityScalingFactor(0.5);

    // set the target pose
    move_group->setPoseTarget(target_pose);
    move_group->setStartStateToCurrentState();
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "OBJECT %s", success ? "" : "FAILED");
    
    // compute the plan and move the robot
    move_group->move();
    sleep(0.5);
    
    // Real and only for prisms: considering the eef the magnet, adjust the
    // position over the object by going up in eef reference system.
    if(isPrism(id) && !simulation)
    {
        geometry_msgs::Pose worldPose = move_group->getCurrentPose().pose;
        geometry_msgs::Pose eefPose;
        tf2::doTransform(worldPose, eefPose, Manipulation::worldtoeef);
        eefPose.position.z += 0.1;
        tf2::doTransform(eefPose, worldPose, Manipulation::eeftoworld);
        
        move_group->setPoseTarget(worldPose);
        move_group->setStartStateToCurrentState();
        success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "OBJECT %s", success ? "" : "FAILED");
        
        // compute the plan and move the robot
        move_group->move();
        sleep(0.5);
    }
    
    // re-set original eef velocity
	move_group->setMaxVelocityScalingFactor(1.0);
    
}

/**
 * Approach the object by going down.
 * @param meters Offset distance.
 */
double Manipulation::goCartesianDown(double meters) {

    // save waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = move_group->getCurrentPose().pose;
    
    // move the robot down of meters
    target_pose.position.z -= meters;
    waypoints.push_back(target_pose);
    
    // set parameters
    move_group->setPlanningTime(10.0);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    move_group->setStartStateToCurrentState();
    
    int count = 0;
	double fraction = 0;
    while(fraction < 0.8 && count < 100) {
        // compute the path
        fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        count++;
    }
    ROS_INFO_NAMED("tutorial", "EEF DOWN (%.2f%% acheived)", fraction * 100.0);
    
    // if fraction is low don't execute the plan
    if(fraction < 0.6)
        return fraction;
    else {
        // plan and do the trajectory
        my_plan.trajectory_ = trajectory;
        move_group->execute(my_plan);
        return fraction;
    }
}

/**
* Place the object on the basket of the mobile robot (Used only after the Final Pose)
* @param meters Offset distance
*/
double Manipulation::goCartesianDownBasket(string load, double meters) {

    // save waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = move_group->getCurrentPose().pose;
    
	if(load == "load1") {
		// move the robot down of meters
		target_pose.position.z -= meters;
		target_pose.position.y += 0.06;
	}
	if(load == "load2") {
		// move the robot down of meters
		target_pose.position.z -= meters;
		target_pose.position.y -= 0.02;
		target_pose.position.x += 0.04;
	}

	waypoints.push_back(target_pose);
    
    // set parameters
    move_group->setPlanningTime(10.0);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    move_group->setStartStateToCurrentState();
    
    int count = 0;
	double fraction = 0;
    while(fraction < 0.8 && count < 100) {
        // compute the path
        fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        count++;
    }
    ROS_INFO_NAMED("tutorial", "EEF DOWN (%.2f%% acheived)", fraction * 100.0);
    
    // if fraction is low don't execute the plan
    if(fraction < 0.6)
        return fraction;
    else {
        // plan and do the trajectory
        my_plan.trajectory_ = trajectory;
        move_group->execute(my_plan);
        return fraction;
    }
}

/**
 * Approach the object by going down (for prism only in real).
 * @param meters Offset distance.
 * @return Fraction of the plan that is feasible.
 */
double Manipulation::goCartesianDownPrism(double meters) {
    
    geometry_msgs::Pose worldPose = move_group->getCurrentPose().pose;
    geometry_msgs::Pose eefPose;
    
    // approach the prism by going down, the trajectory is taken from eef reference system
    tf2::doTransform(worldPose, eefPose, Manipulation::worldtoeef);
    eefPose.position.x += meters;
    tf2::doTransform(eefPose, worldPose, Manipulation::eeftoworld);
    
    // save waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(worldPose);
    
    // set parameters
    move_group->setPlanningTime(10.0);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    move_group->setStartStateToCurrentState();
    
    // compute the path
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "EEF DOWN (%.2f%% acheived)", fraction * 100.0);
    
    // if fraction is low don't execute the plan
    if(fraction < 0.6)
        return fraction;
    else {
        // plan and do the trajectory
        my_plan.trajectory_ = trajectory;
        move_group->execute(my_plan);
        return fraction;
    }
}

/**
 * Pull up the object.
 * @param meters Offset distance
 */
void Manipulation::goCartesianUp(double meters) {

    sleep(0.5);

    move_group->setStartStateToCurrentState();
    
    // save waypoints
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = move_group->getCurrentPose().pose;
    
    // move the robot down of meters
    target_pose.position.z += meters;
    waypoints.push_back(target_pose);
    
    // set parameters
    move_group->setPlanningTime(10.0);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    
    // compute the path
    double fraction = move_group->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("tutorial", "EEF UP (%.2f%% acheived)", fraction * 100.0);
    
    // plan and do the trajectory
    my_plan.trajectory_ = trajectory;
    move_group->execute(my_plan);

}

/**
 * Go through intermediate steps before Final position.
 */
void Manipulation::goToIntermiatePosesOnwards() {

    // Intermediate pose 1
    
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group->setStartStateToCurrentState();
    
    // set joints
    joint_group_positions[0] = 0.5698060989379883;
    joint_group_positions[1] = -1.8648603598224085;
    joint_group_positions[2] = -1.090482536946432;
    joint_group_positions[3] = -1.6997583548175257;
    joint_group_positions[4] = 1.5435876846313477;
    joint_group_positions[5] = 0.008323793299496174;
    
    move_group->setJointValueTarget(joint_group_positions);
    
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "INTERMEDIATE POSE 1 %s", success ? "" : "FAILED");
    
    // compute the plan and move the robot
    move_group->move();
    
    // Intermediate pose 2
    
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group->setStartStateToCurrentState();
    
    // set joints
    joint_group_positions[0] = -0.5766547361956995;
    joint_group_positions[1] = -1.8648365179644983;
    joint_group_positions[2] = -1.0904706160174769;
    joint_group_positions[3] = -1.699782673512594;
    joint_group_positions[4] = 1.5435516834259033;
    joint_group_positions[5] = 0.008287840522825718;
    
    move_group->setJointValueTarget(joint_group_positions);
    
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "INTERMEDIATE POSE 2 %s", success ? "" : "FAILED");
    
    // compute the plan and move the robot
    move_group->move();
    
}

/**
 * Go through intermediate steps before Main position.
 * @param i Is the index of object to detach and remove from collision avoidance
 */
void Manipulation::goToIntermiatePosesBackwards(int i) {
    
    // Intermediate pose 2
    
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group->setStartStateToCurrentState();
    
    // set joints
    joint_group_positions[0] = -0.5766547361956995;
    joint_group_positions[1] = -1.8648365179644983;
    joint_group_positions[2] = -1.0904706160174769;
    joint_group_positions[3] = -1.699782673512594;
    joint_group_positions[4] = 1.5435516834259033;
    joint_group_positions[5] = 0.008287840522825718;
    
    move_group->setJointValueTarget(joint_group_positions);
    
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "INTERMEDIATE POSE 2 %s", success ? "" : "FAILED");
    
    // compute the plan and move the robot
    move_group->move();
    
    // Intermediate pose 1
    
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group->setStartStateToCurrentState();
    
    // set joints
    joint_group_positions[0] = 0.5698060989379883;
    joint_group_positions[1] = -1.8648603598224085;
    joint_group_positions[2] = -1.090482536946432;
    joint_group_positions[3] = -1.6997583548175257;
    joint_group_positions[4] = 1.5435876846313477;
    joint_group_positions[5] = 0.008323793299496174;
    
    move_group->setJointValueTarget(joint_group_positions);
    
    success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "INTERMEDIATE POSE 1 %s", success ? "" : "FAILED");
    
    // compute the plan and move the robot
    move_group->move();
    
}

/**
 * Go to final pose based on the chosen gate.
 * @param load Loading station
 * @param i Index of the object requested by the user
 */
void Manipulation::goToFinalPose(string load, int i) {

    int load_pose = i%2;
    
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    move_group->setStartStateToCurrentState();
    
    if(load=="load1") {
        if(load_pose == 0) {
            // set joints
            joint_group_positions[0] = -0.5970099608050745;
            joint_group_positions[1] = -2.216843907033102;
            joint_group_positions[2] = -1.19104510942568;
            joint_group_positions[3] = -1.2485979239093226;
            joint_group_positions[4] = 1.5399842262268066;
            joint_group_positions[5] = 0.07192033529281616;
            
        }
        if(load_pose == 1) {
            // set joints
            joint_group_positions[0] = -0.6493166128741663;
            joint_group_positions[1] = -1.9344638029681605;
            joint_group_positions[2] = -1.6302769819842737;
            joint_group_positions[3] = -1.0902798811541956;
            joint_group_positions[4] = 1.5439823865890503;
            joint_group_positions[5] = 0.006443393416702747;
            
        }
        //
        move_group->setJointValueTarget(joint_group_positions);
        
        success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "FINAL POSE %s", success ? "" : "FAILED");
        
        // compute the plan and move the robot
        move_group->move();
        
    }
    else if(load=="load2") {
        // set joints
        joint_group_positions[0] = -1.2848594824420374;
        joint_group_positions[1] = -1.8648245970355433;
        joint_group_positions[2] = -1.090482536946432;
        joint_group_positions[3] = -1.699770752583639;
        joint_group_positions[4] = 1.5435277223587036;
        joint_group_positions[5] = 0.008323793299496174;
        
        move_group->setJointValueTarget(joint_group_positions);
        
        success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "INTERMEDIATE POSE 3 %s", success ? "" : "FAILED");
        
        // compute the plan and move the robot
        move_group->move();
        
        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        move_group->setStartStateToCurrentState();
        
        if(load_pose == 0) {
            // set joints
            joint_group_positions[0] = -1.273529354725973;
            joint_group_positions[1] = -1.9209860006915491;
            joint_group_positions[2] = -1.609246079121725;
            joint_group_positions[3] = -1.1246994177447718;
            joint_group_positions[4] = 1.5438865423202515;
            joint_group_positions[5] = 0.006587204523384571;
        }
        if(load_pose == 1) {
            // set joints
            joint_group_positions[0] = -1.502073113118307;
            joint_group_positions[1] = -1.9155662695514124;
            joint_group_positions[2] = -1.5998809973346155;
            joint_group_positions[3] = -1.1395533720599573;
            joint_group_positions[4] = 1.5439344644546509;
            joint_group_positions[5] = 0.006682703737169504;
            
        }
        //
        move_group->setJointValueTarget(joint_group_positions);
        
        success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "FINAL POSE %s", success ? "" : "FAILED");
        
        // compute the plan and move the robot
        move_group->move();
        
    }
    else cout << "Non valid load name!" << endl;
    
    //** detach and remove object from collision avoidance **//
    string obj_id = std::to_string(requestedObjects[i].id[0]);
    
    // detach the object to the arm for collision avoidance
    move_group->detachObject(obj_id);
    
    // remove the object collision of this object from the world
    std::vector<std::string> object_ids;
    object_ids.push_back(obj_id);
    planning_scene_interface->removeCollisionObjects(object_ids);
    
    sleep(1.0);
    
}

/**
 * Recovery function. When working with the magnet it can happen that the object is not grasped
 * correctly. This function check if the object is again on the table, if yes it re-add the
 * collision object and try to grasp the object again with a new sequence.
 * @param i Index of the object requested by the user
 * @return Bool that indicate if there is yet or not the "i" object on the table.
 */
bool Manipulation::objectRecovery(int i) {

	// subscribe topic from Apriltag in order to check if the object i has been grabbed
    boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("tag_detections");

	// id of the object
	int id = requestedObjects[i].id[0];
			
	// retrieve the number of objects in the table
    int n_obj = (msg -> detections).size();

    // check if somewhere there is yet the object picked
	for(int i=0; i < n_obj; i++) {
		if(msg -> detections[i].id[0] == id) {
            geometry_msgs::Pose cameraPose = msg -> detections[i].pose.pose.pose;
            geometry_msgs::Pose worldPose;
            tf2::doTransform(cameraPose, worldPose, Manipulation::cameratobase);

            // if the object is yet on the table (not picked)
            if(worldPose.position.y < 0) {
				
				cout << "The object was not taken correctly" << endl;
                
				//** detach and remove object from collision avoidance **//
				string obj_id = std::to_string(id);

				// detach the object to the arm for collision avoidance
	   			move_group->detachObject(obj_id);
				sleep(0.5);

				std::vector<std::string> object_ids;
				object_ids.push_back(obj_id);

                // retrieve original object collision
				std::map<std::string, moveit_msgs::CollisionObject> obj = planning_scene_interface-> getObjects(object_ids);

				std::map<std::string, moveit_msgs::CollisionObject>::iterator it = obj.begin();

				moveit_msgs::CollisionObject object = it -> second;
				
				// set the correct position and orientation of the object on the table
		       	tf::Quaternion init_quat(worldPose.orientation.x, worldPose.orientation.y, worldPose.orientation.z, worldPose.orientation.w);
		        
		        double roll, pitch, yaw;
		        
		        // get the RPY of the pose
		        tf::Matrix3x3(init_quat).getRPY(roll, pitch, yaw);
		        
		        double x = worldPose.position.x;
				
				// if the object in the table is a CUBE or a CYLINDER
		    	if((id >= 0 && id <= 5) || (id >= 9 && id <= 12)) {
					
		        	// compute a rotation of the pitch and roll
		        	roll = 0;
		        	pitch = 0;
		        	
		        	tf::Quaternion final_quat(0,0,0,1);
		        
		        	// set the new orientation and transform it to quaternion
		        	final_quat.setRPY(roll, pitch, yaw);
		        
		        	// set the orientation
		        	object.primitive_poses[0].orientation.w = final_quat.w();
		        	object.primitive_poses[0].orientation.x = final_quat.x();
		        	object.primitive_poses[0].orientation.y = final_quat.y();
		        	object.primitive_poses[0].orientation.z = final_quat.z();

                    // set the position
		            if(simulation) {
		                if((id >= 0 && id <= 3) || (id >= 9 && id <= 12)) object.primitive_poses[0].position.z = cube_z_sim - 0.03;
		                if(id >= 4 && id <= 5) object.primitive_poses[0].position.z = cyl_z_sim - 0.11;
		            }
		            else {
		                if((id >= 0 && id <= 3) || (id >= 9 && id <= 12)) object.primitive_poses[0].position.z = cube_z_real + computeSlopeHeight(x) + 0.055 - 0.03;
		                if(id >= 4 && id <= 5) object.primitive_poses[0].position.z = cyl_z_real + computeSlopeHeight(x) + 0.055 - 0.11;
		            }
					object.primitive_poses[0].position.x = worldPose.position.x;
					object.primitive_poses[0].position.y = worldPose.position.y;
					
					// samples a new random point, in a circle with a radius of 2 cm and with center the centroid of the apriltag.
					srand((unsigned int)time(NULL));
    				float limit = 0.04;
					float x_off = (((float)rand()/(float)(RAND_MAX)) * limit) - 0.02;
    				float y_off = (((float)rand()/(float)(RAND_MAX)) * limit) - 0.02;
				
					requestedObjects[i].pose.pose.pose.position.x += x_off;
					requestedObjects[i].pose.pose.pose.position.y += y_off;

				}

				// if the object in the table is a TRIANGLE
				if((id >= 6 && id <= 8) || (id >= 13 && id <= 15)) {
				    
				    // compute sin and cosine of yaw (angle in z axis)
				    float result_sin = sin (yaw);
				    float result_cos = cos (yaw);
				    
				    // obtain degree value from rad value
				    float degrees = 0;
				    
				    if((yaw*180)/M_PI>0) degrees = (yaw*180)/M_PI;
				    else degrees = 360 + (yaw*180)/M_PI;
				    
				    // distance from the center of position of Apriltag and true centroid of the prism
				    float distance = 0.031;
				    
				    float y_shift = result_cos * distance;
				    float x_shift = result_sin * distance;
				    
				    // set the position
				    if(degrees<= 90) {
				        worldPose.position.y -= abs(y_shift);
				        worldPose.position.x += abs(x_shift);
				    }
				    if(degrees> 90 && degrees <= 180) {
				        worldPose.position.y += abs(y_shift);
				        worldPose.position.x += abs(x_shift);
				    }
				    if(degrees> 180 && degrees <= 270) {
				        worldPose.position.y += abs(y_shift);
				        worldPose.position.x -= abs(x_shift);
				    }
				    if(degrees> 270 && degrees <= 360) {
				        worldPose.position.y -= abs(y_shift);
				        worldPose.position.x -= abs(x_shift);
				    }
					
					// set the orientation
					object.primitive_poses[0].orientation = worldPose.orientation;
					
					// set the position
		            if(simulation) object.primitive_poses[0].position.z = prism_z_sim - 0.03;
		            else object.primitive_poses[0].position.z = prism_z_real + computeSlopeHeight(x) + 0.052 - 0.03;

					object.primitive_poses[0].position.x = worldPose.position.x;
					object.primitive_poses[0].position.y = worldPose.position.y;

					// samples a new random point, in a circle with a radius of 2 cm and with center the centroid of the apriltag.
					srand((unsigned int)time(NULL));
    				float limit = 0.04;
					float x_off = (((float)rand()/(float)(RAND_MAX)) * limit) - 0.02;
    				float y_off = (((float)rand()/(float)(RAND_MAX)) * limit) - 0.02;

					// the offset in case of prisms is cos(45) * offset
					x_off *= 0.707107;
					y_off *= 0.707107;

					requestedObjects[i].pose.pose.pose.position.x += x_off;
					requestedObjects[i].pose.pose.pose.position.y += y_off;
				}

                // re-add the object to collision avoidance
				object.operation = object.ADD;

				std::vector<moveit_msgs::CollisionObject> collision_objects;
				collision_objects.push_back(object);

				planning_scene_interface->addCollisionObjects(collision_objects);
				cout << "Re-added collision object into the scene" << endl;
				return true;
			}
		}
	}
	
	return false;

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// FUNCTIONS FOR OBTAIN AND TRANSFORM POSES /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Obtain poses of objects on the table.
 */
void Manipulation::obtainCollisionObjects() {
    
    // subscribe topic from Apriltag in order to obtain all position of the object in the table, and apply to them collision avoidance
    boost::shared_ptr<apriltag_ros::AprilTagDetectionArray const> msg = ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("tag_detections", ros::Duration(10.0));
    
	if(msg != NULL) {
		// compute the pose_camera that is pose of the object in the reference system of the camera
		// pose_world is the pose trasformed in reference system of /world
		geometry_msgs::PoseStamped pose_world;
		geometry_msgs::PoseStamped pose_camera;
		
		// retrieve the number of objects in the table
		int n_obj = (msg -> detections).size();
		
		// struct with ID and POSE of objects in the table
		for(int i=0; i < n_obj; i++) collisionObjects.push_back(msg -> detections[i]);
		
		// obtain for every object a transformed POSE (in "world" reference system)
		for(int i=0; i< n_obj; i++) {
		    pose_camera.header = collisionObjects[i].pose.header;
		    pose_camera.pose = collisionObjects[i].pose.pose.pose;
		    tf2::doTransform(pose_camera, pose_world, Manipulation::cameratobase);
		    collisionObjects[i].pose.header = pose_world.header;
		    collisionObjects[i].pose.pose.pose = pose_world.pose;
		    
		    // set fixed height to the objects based on their type
		    string type = obtainObjectType(collisionObjects[i].id[0]);
		    double x = collisionObjects[i].pose.pose.pose.position.x;
		    if(simulation) {
		        if(type == "cube") collisionObjects[i].pose.pose.pose.position.z = cube_z_sim;
		        if(type == "hexagon") collisionObjects[i].pose.pose.pose.position.z = cyl_z_sim;
		        if(type == "prism") collisionObjects[i].pose.pose.pose.position.z = prism_z_sim;
		    }
		    else {
		        // fixed height + correction due to the slope of the table on x axis + magnet eef_link height
		        if(type == "cube") collisionObjects[i].pose.pose.pose.position.z = cube_z_real + computeSlopeHeight(x) + 0.055;
		        if(type == "hexagon") collisionObjects[i].pose.pose.pose.position.z = cyl_z_real + computeSlopeHeight(x) + 0.055;
		        if(type == "prism") collisionObjects[i].pose.pose.pose.position.z = prism_z_real + computeSlopeHeight(x) + 0.052;
		    }
		}
    }
}

/**
 * Obtain poses requested by the user.
 */
void Manipulation::obtainRequestedObjects() {
    
    // subscribe topic from Apriltag in order to obtain all position of the object in the table, and apply to them collision avoidance
    boost::shared_ptr<hw4::msg1 const> msg =
    ros::topic::waitForMessage<hw4::msg1>("/ur5/poses", ros::Duration(4.0));

    if(msg != NULL) {
    	// compute the pose_camera that is pose of the object in the reference system of the camera
    	// pose_world is the pose trasformed in reference system of /world
    	geometry_msgs::PoseStamped pose_world;
    	geometry_msgs::PoseStamped pose_camera;
    
    	// retrieve the number of objects in the table
    	int n_obj = (msg -> detections).size();
    
    	// struct with ID and POSE of objects in the table
    	for(int i=0; i < n_obj; i++) requestedObjects.push_back(msg -> detections[i]);
    
    	// obtain for every object a transformed POSE (in "world" reference system)
    	for(int i=0; i< n_obj; i++) {
        	pose_camera.header = requestedObjects[i].pose.header;
        	pose_camera.pose = requestedObjects[i].pose.pose.pose;
        	tf2::doTransform(pose_camera, pose_world, Manipulation::cameratobase);
        	requestedObjects[i].pose.header = pose_world.header;
        	requestedObjects[i].pose.pose.pose = pose_world.pose;
            
            // set fixed height to the objects based on their type
            string type = obtainObjectType(requestedObjects[i].id[0]);
            double x = requestedObjects[i].pose.pose.pose.position.x;
            if(simulation) {
                if(type == "cube") requestedObjects[i].pose.pose.pose.position.z = cube_z_sim;
                if(type == "hexagon") requestedObjects[i].pose.pose.pose.position.z = cyl_z_sim;
                if(type == "prism") requestedObjects[i].pose.pose.pose.position.z = prism_z_sim;
            }
            else {
                // fixed height + correction due to the slope of the table on x axis + magnet eef_link height - distance to object centroid
                if(type == "cube") requestedObjects[i].pose.pose.pose.position.z = cube_z_real + computeSlopeHeight(x) + 0.055 - 0.03;
                if(type == "hexagon") requestedObjects[i].pose.pose.pose.position.z = cyl_z_real + computeSlopeHeight(x) + 0.055 - 0.11;
                if(type == "prism") requestedObjects[i].pose.pose.pose.position.z = prism_z_real + computeSlopeHeight(x) + 0.052 - 0.03;
            }
    	}
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// FUNCTIONS FOR CONTROLLING THE GRIPPER /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Function for Gripper opening.
 * @param pub Publisher for Gripper control
 * @param str Message for Gripper control
 */
void Manipulation::openGripper(ros::Publisher& pub, robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput& str) {
    
    // OPEN GRIPPER
    str.rACT = 1;
    str.rMOD = 0;
    str.rGTO = 1;
    str.rATR = 0;
    str.rGLV = 0;
    str.rICF = 0;
    str.rICS = 0;
    str.rPRA = 0;
    str.rSPA = 200;
    str.rFRA = 0;
    str.rPRB = 0;
    str.rSPB = 0;
    str.rFRB = 0;
    str.rPRC = 0;
    str.rSPC = 0;
    str.rFRC = 0;
    str.rPRS = 0;
    str.rSPS = 0;
    str.rFRS = 0;
    
    // publish only one message:
    // command to open the gripper
    while(ros::ok()) {
        // publish the message when there is a subscriber
        if(pub.getNumSubscribers() > 0) {
            pub.publish(str);
            break;
        }
    } // end while
    
    // wait the aperture of the gripper
    sleep(3.0);
    cout << "Gripper opening succeded" << endl;
}

/**
 * Function for Gripper closing.
 * @param pub Publisher for Gripper control
 * @param str Message for Gripper control
 */
void Manipulation::closeGripper(ros::Publisher& pub, robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput& str) {
    
    // CLOSE GRIPPER
    str.rACT = 1;
    str.rMOD = 0;
    str.rGTO = 1;
    str.rATR = 0;
    str.rGLV = 0;
    str.rICF = 0;
    str.rICS = 0;
    str.rPRA = 250;
    str.rSPA = 200;
    str.rFRA = 200;
    str.rPRB = 0;
    str.rSPB = 0;
    str.rFRB = 0;
    str.rPRC = 0;
    str.rSPC = 0;
    str.rFRC = 0;
    str.rPRS = 0;
    str.rSPS = 0;
    str.rFRS = 0;
    
    // publish only one message:
    // command to close the gripper
    while(ros::ok()) {
        // publish the message when there is a subscriber
        if(pub.getNumSubscribers() > 0) {
            pub.publish(str);
            break;
        }
    } // end while
    
    // wait the closure of the gripper
    sleep(3.0);
    cout << "Gripper closing succeded" << endl;
}

/**
 * Function for Magnet activation/deactivation.
 * @param srv Service for Magnet control
 * @param set_io Message for Magnet control
 * @param operation Type of operation
 */
void Manipulation::magnetControl(ros::ServiceClient& srv, ur_msgs::SetIO& set_io, std::string operation) {
    
    int input_command = 0;
    
    if (operation == "magnet_on") input_command = 1;
    
    set_io.request.fun = set_io.request.FUN_SET_DIGITAL_OUT;
    set_io.request.pin = 0;
    set_io.request.state = input_command;
    
    if (srv.call(set_io))
        ROS_INFO_STREAM("Operation " + operation + " succeded");
    else
        ROS_ERROR_STREAM("Failed to call Robot Set I/O service for operation " + operation);
    
    sleep(2.0);
    
}

/**
 * Function for Link attach/detach.
 * @param srv Service for Link attacher
 * @param i Index of the object requested by the user
 */
void Manipulation::linkAttach(ros::ServiceClient& srv, int i) {

    hw4::Attach set_link;
	// set the model and link of the robot
    set_link.request.model_name_1 = "ar_ur5";
    set_link.request.link_name_1 = "wrist_2_link";

	int index = requestedObjects[i].id[0];

	// model and link of the object requested
	string model_type = model_types[index];
	string link_type = link_types[index];

	// set the model and link of the object
    set_link.request.model_name_2 = model_type;
    set_link.request.link_name_2 = link_type;

	if (srv.call(set_link))
        ROS_INFO_STREAM("Link Attach/Detach Succeded");
    else
        ROS_ERROR_STREAM("Link Attach/Detach Failed");
    
    sleep(1.0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// FUNCTIONS FOR COLLISION AVOIDANCE ///////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Function that add object collision avoidance to object in the table.
 */
void Manipulation::applyCollisionObjects() {
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    shape_msgs::SolidPrimitive primitive;
    
    std::string s = "";
    int id = 0;
    
    // for every object in the table
    for(int i=0; i<collisionObjects.size(); i++) {
        
        // from its id check if it is a cube, triangle or hexagon
        id = collisionObjects[i].id[0];
        s = std::to_string(id);
        
        // set collision object header
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "world";
        
        // if the object in the table is a CUBE
        if((id >= 0 && id <= 3) || (id >= 9 && id <= 12)) {
            
            // set collision object id with its id (from 0 to 15)
            collision_object.id = s;
            
            // CUBE
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            
            // SET DIMENSIONS
            primitive.dimensions[0] = 0.1;
            primitive.dimensions[1] = 0.1;
            primitive.dimensions[2] = 0.1;
            
            // SET POSE
            geometry_msgs::Pose box_pose;
            box_pose.position = collisionObjects[i].pose.pose.pose.position;
            
            // obtain a pose that have EEF facing down to grasp
            tf::Quaternion init_quat(collisionObjects[i].pose.pose.pose.orientation.x, collisionObjects[i].pose.pose.pose.orientation.y, collisionObjects[i].pose.pose.pose.orientation.z, collisionObjects[i].pose.pose.pose.orientation.w);
            
            double roll, pitch, yaw;
            
            // get the RPY of the pose
            tf::Matrix3x3(init_quat).getRPY(roll, pitch, yaw);
            
            // compute a rotation of the pitch and roll
            roll = 0;
            pitch = 0;
            
            tf::Quaternion final_quat(0,0,0,1);
            
            // set the new orientation and transform it to quaternion
            final_quat.setRPY(roll, pitch, yaw);
            
            // set the orientation
            box_pose.orientation.w = final_quat.w();
            box_pose.orientation.x = final_quat.x();
            box_pose.orientation.y = final_quat.y();
            box_pose.orientation.z = final_quat.z();
			box_pose.position.z -= 0.03;
            
            // ADD OBJECT TO COLLISION AVOIDANCE
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            
            collision_objects.push_back(collision_object);
            
        } // end if
        
        // if the object in the table is a HEXAGON
        if(id >= 4 && id <= 5) {
            
            // set collision object id with its id (from 0 to 15)
            collision_object.id = s;
            
            // CYLINDER
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            
            // SET DIMENSIONS
            primitive.dimensions[0] = 0.20;
            primitive.dimensions[1] = 0.05;
            
            // SET POSE
            geometry_msgs::Pose box_pose;
            box_pose.position = collisionObjects[i].pose.pose.pose.position;
            box_pose.position.z -= 0.11;
            
            // obtain a pose that have EEF facing down to grasp
            tf::Quaternion init_quat(collisionObjects[i].pose.pose.pose.orientation.x, collisionObjects[i].pose.pose.pose.orientation.y, collisionObjects[i].pose.pose.pose.orientation.z, collisionObjects[i].pose.pose.pose.orientation.w);
            
            double roll, pitch, yaw;
            
            // get the RPY of the pose
            tf::Matrix3x3(init_quat).getRPY(roll, pitch, yaw);
            
            // compute a rotation of the pitch and roll
            roll = 0;
            pitch = 0;
            
            tf::Quaternion final_quat(0,0,0,1);
            
            // set the new orientation and transform it to quaternion
            final_quat.setRPY(roll, pitch, yaw);
            
            // set the orientation
            box_pose.orientation.w = final_quat.w();
            box_pose.orientation.x = final_quat.x();
            box_pose.orientation.y = final_quat.y();
            box_pose.orientation.z = final_quat.z();
            
            // ADD OBJECT TO COLLISION AVOIDANCE
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            
            collision_objects.push_back(collision_object);
            
        } // end if
        
		// if the object in the table is a TRIANGLE
		if((id >= 6 && id <= 8) || (id >= 13 && id <= 15)) {

		// set collision object id with its id (from 0 to 15)
		collision_object.id = s;

		// TRIANGLE
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);

		// SET DIMENSIONS
		primitive.dimensions[0] = 0.1;
		primitive.dimensions[1] = 0.1;
		primitive.dimensions[2] = 0.1;

		// SET POSE
		geometry_msgs::Pose box_pose;

		box_pose.orientation = collisionObjects[i].pose.pose.pose.orientation;
		// obtain a pose that have EEF facing down to grasp
        tf::Quaternion init_quat(collisionObjects[i].pose.pose.pose.orientation.x, collisionObjects[i].pose.pose.pose.orientation.y, collisionObjects[i].pose.pose.pose.orientation.z, collisionObjects[i].pose.pose.pose.orientation.w);
            
            double roll, pitch, yaw;
            
            // get the RPY of the pose
            tf::Matrix3x3(init_quat).getRPY(roll, pitch, yaw);
            
            // compute sin and cosine of yaw (angle in z axis)
            float result_sin = sin (yaw);
            float result_cos = cos (yaw);
            
            // obtain degree value from rad value
            float degrees = 0;
            
            if((yaw*180)/M_PI>0) degrees = (yaw*180)/M_PI;
            else degrees = 360 + (yaw*180)/M_PI;
            
            // distance from the center of position of Apriltag and true center of the prism
            float distance = 0.031;
            
            float y_shift = result_cos * distance;
            float x_shift = result_sin * distance;
            
            // set the position
            if(degrees<=90) {
                collisionObjects[i].pose.pose.pose.position.y -= abs(y_shift);
                collisionObjects[i].pose.pose.pose.position.x += abs(x_shift);
            }
            if(degrees>90 && degrees <= 180) {
                collisionObjects[i].pose.pose.pose.position.y += abs(y_shift);
                collisionObjects[i].pose.pose.pose.position.x += abs(x_shift);
            }
            if(degrees>180 && degrees <= 270) {
                collisionObjects[i].pose.pose.pose.position.y += abs(y_shift);
                collisionObjects[i].pose.pose.pose.position.x -= abs(x_shift);
            }
            if(degrees>270 && degrees <= 360) {
                collisionObjects[i].pose.pose.pose.position.y -= abs(y_shift);
                collisionObjects[i].pose.pose.pose.position.x -= abs(x_shift);
            }
		box_pose.position = collisionObjects[i].pose.pose.pose.position;
		
		box_pose.position.z -= 0.03;

		// ADD OBJECT TO COLLISION AVOIDANCE
		collision_object.primitives.push_back(primitive);
		collision_object.primitive_poses.push_back(box_pose);
		collision_object.operation = collision_object.ADD;

		collision_objects.push_back(collision_object);

		}

    } // end for
    
    planning_scene_interface->applyCollisionObjects(collision_objects);
	cout << "Collision Object applied into the scene" << endl;

}

/**
 * Attach object collision to arm.
 * @param i Index of the object requested by the user
 * @param offset Distance in z to re-add object collision
 */
void Manipulation::attachObjectCollision(int i, double offset) {
    
    //** modify position of the object from collision avoidance **//
    string obj_id = std::to_string(requestedObjects[i].id[0]);
    std::vector<std::string> object_ids;
    object_ids.push_back(obj_id);
    
    std::map<std::string, moveit_msgs::CollisionObject> obj = planning_scene_interface-> getObjects(object_ids);
    
    std::map<std::string, moveit_msgs::CollisionObject>::iterator it = obj.begin();
    
    moveit_msgs::CollisionObject object = it -> second;
    
    object.primitive_poses[0].position.z += offset;
    object.operation = object.ADD;
    
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(object);
    
    planning_scene_interface->applyCollisionObjects(collision_objects);
    
    // attach the object to the arm for collision avoidance
    move_group->attachObject(obj_id);
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// UTILITY FUNCTIONS ///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Clean the world from the collision objects.
 */
void Manipulation::cleanWorld() {
    
    sleep(1.0);
    vector<string> objectsInTable = planning_scene_interface -> getKnownObjectNames();
    planning_scene_interface->removeCollisionObjects(objectsInTable);
    cout << "World cleaned" << endl;
}

/**
 * Get the number of requested objects.
 * @return Number of objects
 */
int Manipulation::getNumberRequestedObjects() {
    
    return Manipulation::requestedObjects.size();
}

/**
 * Get requested objects.
 * @return objects
 */
vector<apriltag_ros::AprilTagDetection> Manipulation::getRequestedObjects() {
    return requestedObjects;
}

/**
 * set requested objects.
 */
void Manipulation::setRequestedObjects(vector<apriltag_ros::AprilTagDetection> obj) {
    Manipulation::requestedObjects = obj;
}

/**
 * Obtain the object id.
 * @param i Index of the object requested by the user
 * @return ID of the object
 */
int Manipulation::obtainIdObject(int i) {
    return requestedObjects[i].id[0];
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// UTILS FUNCTIONS FOR FUNCTIONS OF THE CLASS ///////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Linear function to adjust the position of z due to the inclination of the table in real.
 * @param current_x
 * @return Distance in meters
 */
double computeSlopeHeight(double current_x) {
    double min = 0.44;
    double max = -0.44;
    double coeff = abs(current_x - min)/abs(max-min);
    return coeff*0.009;
}

/**
 * From the id, return if the objets is a Prism.
 * @param i Index of the object requested by the user
 * @return Is Prism or not
 */
bool isPrism(int id) {
    // if the object in the table is a PRISM
    if((id >= 6 && id <= 8) || (id >= 13 && id <= 15)) return true;
    else return false;
}

/**
 * From the id, return the type of the object.
 * @param i Index of the object requested by the user
 * @return The type of the object
 */
string obtainObjectType(int id) {
    
    // if the object in the table is a CUBE
    if((id >= 0 && id <= 3) || (id >= 9 && id <= 12)) return "cube";
    // if the object in the table is a HEXAGON
    if(id >= 4 && id <= 5) return "hexagon";
    // if the object in the table is a PRISM
    if((id >= 6 && id <= 8) || (id >= 13 && id <= 15)) return "prism";
}




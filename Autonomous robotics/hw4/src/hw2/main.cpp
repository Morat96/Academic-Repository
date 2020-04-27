#include <ros/ros.h>
// Manipulation Class
#include "manipulation.h"

using namespace std;

// select the number of objects to load on the mobile robot
void selectNumberOfSequences(int& seq, int& marrtinoObjects, int& objectsToLoad) {
    
    if(marrtinoObjects <= seq) objectsToLoad = marrtinoObjects;
    // if the user requests an excessive number of objects, loads all the required objects remaining on the table
    if(marrtinoObjects > seq) objectsToLoad = seq;
}

// function that starts the manipulation sequence after receiving a call to the "hw4" service
bool callback(hw4::srv1::Request &req, hw4::srv1::Response &res, Manipulation* move_it, int simulation, int gripper_enable,
				ros::Publisher pub, ros::ServiceClient srv, ros::ServiceClient srv_attach, ros::ServiceClient srv_detach) {
    
	robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput str;
 	ur_msgs::SetIO set_io;
	hw4::Attach set_link;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    move_it -> goToMainPosition();
    move_it -> goToExternalPosition();
    // create object collision
    move_it -> obtainCollisionObjects();
    move_it -> applyCollisionObjects();
    // obtain requested objects by the user
    move_it -> obtainRequestedObjects();
    
    // get the number of objects requested by the user
    int seq = move_it -> getNumberRequestedObjects();
    // get the number of objects requested at the start position of marrtino by end-user
    int marrtinoObjects = req.objects;
    // number of repetition of the sequence
    int objectsToLoad;
    
    selectNumberOfSequences(seq, marrtinoObjects, objectsToLoad);
    
    for(int i=0; i < objectsToLoad; i++) {

        // when working with the magnet it can happen that the object is not grasped
        // correctly, the cycle tries to grasp the object by repeating the sequence
        bool objectNotTaken = true;
        while(objectNotTaken) {
            move_it -> goToMainPosition();
            int targetID = move_it -> obtainIdObject(i);
            if(simulation) {
                if(gripper_enable) {
                    move_it -> goToTargetPosition(i);
                    // Gripper opening
                    move_it -> openGripper(pub, str);
                    move_it -> goCartesianDown(0.13);
                    // Gripper closing
                    move_it -> closeGripper(pub, str);
                    // attach the object
                    move_it -> linkAttach(srv_attach,i);
                }
                else {
                    // if is a Prism
                    if((targetID >= 6 && targetID <= 8) || (targetID >= 13 && targetID <= 15)) {
                        move_it -> goToTargetPosition(i);
                        sleep(1.0);
                        move_it -> goCartesianDown(0.07);
                    }
                    // if is a Hexagon or a Cube
                    else
                    {
                        move_it -> goToTargetPosition(i);
                        sleep(1.0);
                        move_it -> goCartesianDown(0.21);
                    }
                }
            }
            else {
                // if is a Prism
                if((targetID >= 6 && targetID <= 8) || (targetID >= 13 && targetID <= 15)) {
                    move_it -> goToTargetPosition(i);
                    
                    bool done = false;
                    
                    sleep(1.0);
                    // approach the prism by going down
                    double fraction = move_it -> goCartesianDownPrism(0.02);
                    sleep(1.0);
                    //iterate until the object is reached
                    while(!done)
                    {
                        fraction = move_it -> goCartesianDownPrism(0.004);
                        if(fraction < 0.3) done = true;
                        sleep(0.2);
                    }
                }
                // if is a Hexagon or a Cube
                else
                {
                    move_it -> goToTargetPosition(i);
                    sleep(1.0);
                    // approach to the object
                    double fraction = move_it -> goCartesianDown(0.14);
                    sleep(1.0);
                    
                    // iterate until the object is reached
                    bool done = false;
                    
                    while(!done)
                    {
                        fraction = move_it -> goCartesianDown(0.004);
                        if(fraction < 0.3) done = true;
                        sleep(0.2);
                    }
                }
                
            }
            
            // *************** FUNCTIONS FOR OBJECTS ATTACH ***************** //
            
            // Magnet
            if(!simulation) move_it -> magnetControl(srv, set_io, "magnet_on");
            
            // Link Attach object
            if(!gripper_enable && simulation) move_it -> linkAttach(srv_attach,i);
            
            sleep(1.0);
            
            // ************************************************************** //
            
            move_it -> goCartesianUp(0.15);
            move_it -> attachObjectCollision(i, 0.14);
            move_it -> goCartesianUp(0.04);
            move_it -> goToMainPosition();
            move_it -> goToIntermiatePosesOnwards();
            objectNotTaken = move_it -> objectRecovery(i);
            sleep(1.0);
        }
        // select the correct loading station
        move_it -> goToFinalPose(req.station, i);
		if(!gripper_enable) double fraction = move_it -> goCartesianDownBasket(req.station, 0.04);
        
        // *************** FUNCTIONS FOR OBJECTS DETACH ***************** //
        
        // Magnet
        if(!simulation) move_it -> magnetControl(srv, set_io, "magnet_off");
        
        // Gripper
        if(gripper_enable && simulation) move_it -> openGripper(pub, str);
        
        // Link detach object
        if(simulation) move_it -> linkAttach(srv_detach,i);
        
        // ************************************************************** //
        
        move_it -> goToIntermiatePosesBackwards(i);
        
    }
    
    // clean the scene from the remain collision objects
    move_it -> cleanWorld();
    
    vector<apriltag_ros::AprilTagDetection> remainingObjects = move_it -> getRequestedObjects();
    remainingObjects.erase(remainingObjects.begin(),remainingObjects.begin() + objectsToLoad);
    move_it -> setRequestedObjects(remainingObjects);
    cout << "Remaining objects to load " << move_it -> getNumberRequestedObjects() << endl;
	
    spinner.stop();
    
    // return the number of objects still on the table as response to the service
    res.objOnTable = seq - objectsToLoad;
    ROS_INFO("sending back response: %d remaining objects on the table", res.objOnTable);
    
    return true;
}

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "move");
    ros::NodeHandle n;
    
    // get execution environment: simulation or real
    // default value: simulation = 1
    int simulation;
    n.getParam("move/simulation", simulation);
	if(simulation) cout << "Simulation environment" << endl;
	else cout << "Real environment" << endl;
    
    // use of the gripper: true or false
    // default value: gripper_enable = 1
    int gripper_enable;
    n.getParam("move/gripper_enable", gripper_enable);
    if(gripper_enable) cout << "Using the Gripper" << endl;
    else cout << "Using the Magnet" << endl;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // set planning group
    static const std::string PLANNING_GROUP = "manipulator";
    
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface *move_point;
    move_point = &move_group;

    // set planning interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::PlanningSceneInterface *planning_scene;
    planning_scene = &planning_scene_interface;
    
    // Raw pointers are frequently used to refer to the planning group for improved performance
    const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    // set PUBLISHER for Gripper
    
    ros::Publisher pub = n.advertise<robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput>("/left_hand/command", 1000);

    // set SERVICE for Magnet
    
    ros::ServiceClient srv = n.serviceClient<ur_msgs::SetIO>("/ur5/ur_driver/set_io");

	// set SERVICE for Link Attacher
    
    ros::ServiceClient srv_attach = n.serviceClient<hw4::Attach>("/link_attacher_node/attach");
    ros::ServiceClient srv_detach = n.serviceClient<hw4::Attach>("/link_attacher_node/detach");

    // vectors for poses
    vector<apriltag_ros::AprilTagDetection> objectsCollision;
    vector<apriltag_ros::AprilTagDetection> objectsRequested;
    
    Manipulation *move_it = new Manipulation(move_point, planning_scene, joint_model_group, objectsCollision, objectsRequested, simulation, gripper_enable);
    
    spinner.stop();
    
    ros::MultiThreadedSpinner spinner_service(2); // two threads
    
    ros::ServiceServer service = n.advertiseService<hw4::srv1::Request, hw4::srv1::Response>("hw4", boost::bind(&callback, _1, _2, move_it, simulation, gripper_enable, pub, srv, srv_attach, srv_detach));

    cout << "Waiting request to start the sequence " << endl;
    spinner_service.spin();
 
    return 0;
}

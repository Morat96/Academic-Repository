#include "ObjectDetector.h"

using namespace std;

/**
* @brief Class constructor
*/
ObjectDetector::ObjectDetector(ros::NodeHandle n, vector<apriltag_ros::AprilTagDetection> det, std::vector<std::string> req_obj, string frame_id[16])
{
    ObjectDetector::n = n;
    ObjectDetector::det = det;
    ObjectDetector::req_obj = req_obj;
	ObjectDetector::isSubscribed = false;

    for(int i=0; i<16; i++) ObjectDetector::frame_id[i] = frame_id[i];
    
    // subscribe from Apriltag
    sub = n.subscribe("tag_detections", 1000, &ObjectDetector::callback, this);
 
}

/**
* @brief callback function for Apriltag subscription
* @param msg Message of Apriltag poses
*/
void ObjectDetector::callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    ObjectDetector::det.clear();
    
    std::vector<int> index;
    
    // find the Marker ID comparing user frame ID and the array
    // and save the IDs in a vector
    for(int i=0; i<req_obj.size(); i++) {
        for(int j=0;j<16;j++)
        {
            if(frame_id[j]==req_obj[i]) index.push_back(j);
        }
    }
    
    // create and open a file text
    std::ofstream poses (ros::package::getPath("hw4") + "/poses.txt");

    apriltag_ros::AprilTagDetection detection;
    int n_obj;

    // # of objects on the table
    n_obj = (msg -> detections).size();
    
    // for every object requested by the user, print the informations about position
    // and orientation only for objects that are on the table
    for(int i=0; i<index.size(); i++) {
        // check if the current object id is on the table
        for(int j=0; j<n_obj; j++) {
            // get current detection
            detection = msg -> detections[j];
            int32_t id = detection.id[0];
            
            if(index[i]==id) {
                
                // Console output
                cout << "Frame ID: " << frame_id[id] << endl;
                cout << "Marker ID: " << id << endl;
                cout << "POSE:" << endl;
                cout << "POSITION:" << endl;
                cout << "x: " << detection.pose.pose.pose.position.x << endl;
                cout << "y: " << detection.pose.pose.pose.position.y << endl;
                cout << "z: " << detection.pose.pose.pose.position.z << endl;
                cout << "ORIENTATION:" << endl;
                cout << "x: " << detection.pose.pose.pose.orientation.x << endl;
                cout << "y: " << detection.pose.pose.pose.orientation.y << endl;
                cout << "z: " << detection.pose.pose.pose.orientation.z << endl;
                cout << "w: " << detection.pose.pose.pose.orientation.w << endl;
                
                // for check compute the RPY orientation
                double roll, pitch, yaw;
                tf::Quaternion quat(detection.pose.pose.pose.orientation.x, detection.pose.pose.pose.orientation.y,
                                    detection.pose.pose.pose.orientation.z, detection.pose.pose.pose.orientation.w);
                tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                
                std::cout << "Roll: " << roll << " - Pitch: " << pitch << " - Yaw: " << yaw << "\n" << endl;
                
                // File output
                poses << "Frame ID: " << frame_id[id] << "\n";
                poses << "Marker ID: " << id << "\n";
                poses << "POSE" << "\n" << "Position:" << "\n";
                poses << "x: " << detection.pose.pose.pose.position.x << "\n";
                poses << "y: " << detection.pose.pose.pose.position.y << "\n";
                poses << "z: " << detection.pose.pose.pose.position.z << "\n";
                poses  << "Orientation:" << "\n";
                poses << "x: " << detection.pose.pose.pose.orientation.x << "\n";
                poses << "y: " << detection.pose.pose.pose.orientation.y << "\n";
                poses << "z: " << detection.pose.pose.pose.orientation.z << "\n";
                poses << "w: " << detection.pose.pose.pose.orientation.w << "\n\n";
                
                // vector
                ObjectDetector::det.push_back(detection);
                
            }
        }
        
        
    }
    poses.close();
	ObjectDetector::isSubscribed = true;
}

/**
* @brief get vector of poses
*/
vector<apriltag_ros::AprilTagDetection> ObjectDetector::getDetections() {
    return ObjectDetector::det;
}

/**
* @brief get vector of requested objects by the user
*/
vector<std::string> ObjectDetector::getRequestedObjects() {
    return ObjectDetector::req_obj;
}

/**
* @brief check if there is a subscription
*/
bool ObjectDetector::getIsSubscribed() {
	return ObjectDetector::isSubscribed;
}






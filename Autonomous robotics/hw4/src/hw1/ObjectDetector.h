#ifndef ObjectDetector_
#define ObjectDetector_

#include <stdio.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <std_msgs/Header.h>
#include "hw4/msg1.h"
// RPY
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <vector>

using namespace std;

class ObjectDetector
{
public:

    /**
    * @brief Class constructor
    */
    ObjectDetector(ros::NodeHandle n, vector<apriltag_ros::AprilTagDetection> det, std::vector<std::string> req_obj, string frame_id[16]);

    /**
    * @brief callback function for Apriltag subscription
	* @param msg Message of Apriltag poses
    */
    void callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

    /**
    * @brief get vector of poses
    */
    vector<apriltag_ros::AprilTagDetection> getDetections();

	/**
    * @brief get vector of requested objects by the user
	*/
    std::vector<std::string> getRequestedObjects();

    /**
    * @brief check if there is a subscription
    */
	bool getIsSubscribed();
    
private:

    vector<apriltag_ros::AprilTagDetection> det;
    std::vector<std::string> req_obj;
    string frame_id[16];
    ros::NodeHandle n;
    ros::Subscriber sub;
	bool isSubscribed;
};


#endif /* ObjectDetector */

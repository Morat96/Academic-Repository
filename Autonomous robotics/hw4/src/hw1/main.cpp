#include "ros/ros.h"
// ObjectDetector Class
#include "ObjectDetector.h"

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "rec");
    ros::NodeHandle n;
    
    // obtain objects requested by user from command line
    std::vector<std::string> req_obj(argv+1, argv + argc);
    // vector of poses of objects requested
    vector<apriltag_ros::AprilTagDetection> detections;
    string frame_id[16] = {"red_cube_0", "red_cube_1", "red_cube_2", "red_cube_3", "yellow_cyl_0",
        "yellow_cyl_1", "green_prism_0", "green_prism_1", "green_prism_2", "blue_cube_0",
        "blue_cube_1", "blue_cube_2", "blue_cube_3", "red_prism_0", "red_prism_1", "red_prism_2"};
    
    // instance of class Sub which subscribe Apriltag topic
    ObjectDetector *Detections = new ObjectDetector(n, detections, req_obj, frame_id);

	// create a publisher
    ros::Publisher pub = n.advertise<hw4::msg1>("/ur5/poses", 1000);

	// create msg
    hw4::msg1 msg;

	// publish objects poses twice a second
    ros::Rate loop_rate(2);
    
    ros::spinOnce();

    while(ros::ok()) {

		// publish a message only if there is a subscribe to Apriltag
        if(Detections -> getIsSubscribed()) {

        // clear the message from previous poses
        msg.detections.clear();

        // update msg with value of objects poses
        for (int i = 0; i < Detections -> getDetections().size(); i++) {
            msg.detections.push_back(Detections -> getDetections()[i]);
        }

        // publish the message
        pub.publish(msg);
    	}
	
        ros::spinOnce();

        loop_rate.sleep();
    }
    
return 0;
    
}

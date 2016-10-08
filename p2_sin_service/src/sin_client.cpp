// sin_client by dgp34
// prompts for amplitude and frequency as part of service message
#include<iostream>
#include<ros/ros.h> 
#include<p2_sin_service/SinMessage.h>

int main(int argc, char **argv) {
    // initialize sin_client
    ros::init(argc, argv, "sin_client");

    // initialize node handle
    ros::NodeHandle n;

    // attach client to sin_service (see sin_commander.cpp)
    ros::ServiceClient client = n.serviceClient<p2_sin_service::SinMessage>("sin_service");

    // create a service object
    p2_sin_service::SinMessage srv;

    while (ros::ok()) {

        // receive amplitude and frequency, move to service request
        std::cout << "\nWhat is your desired amplitude? ";
        std::cin >> srv.request.amplitude;
        std::cout << "What is your desired frequency? ";
        std::cin >> srv.request.frequency;

        // Call the client with the requested values
      	if (client.call(srv)){
            ROS_INFO("Call completed?: %d", srv.response.completed);
        } else {
    	    ROS_ERROR("Failed to call sin_service");
            return 1;
        }
    }
    return 0; // should never get here, unless roscore dies 
} 

// sin_commander by dgp34
// advertises:  "sin_service"
// publisher to: "vel_cmd"
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<math.h>
#include<p2_sin_service/SinMessage.h>


// global variables
std_msgs::Float64 g_amp;
std_msgs::Float64 g_freq; 
std_msgs::Float64 g_sin_vel_cmd;

// callback for sin_service which sets amplitude and frequency, given a request
bool callback(p2_sin_service::SinMessageRequest& request, p2_sin_service::SinMessageResponse& response) {
    ROS_INFO("Desired amplitude is: %f", request.amplitude);
    ROS_INFO("Desired frequency is: %f", request.frequency);
    // place received data in appropriate global variables
    g_amp.data = request.amplitude;
    g_freq.data = request.frequency;

    // let user know that the service was carried out
    response.completed = true;
}


int main(int argc, char **argv) {

    // name node sin_commander
    ros::init(argc, argv, "sin_commander"); 
    
    // node handle
    ros::NodeHandle nh;

    // create service for amplitude and velocity
    ros::ServiceServer service = nh.advertiseService("sin_service", callback);

    // create publisher for velocity 
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd", 1);

    // specify 10Hz controller sample rate (pretty slow, but illustrative)
    double dt_controller = 0.10;
    double sample_rate = 1.0 / dt_controller;

    // use to regulate loop rate
    ros::Rate naptime(sample_rate);

    // initialize 0 radians
    double radians = 0.0;

    while (ros::ok()) {
        // increment radians by pi
	// radians increase by Pi/2 every second
        radians = radians + dt_controller * 3.14159265 / 2;
        
        // determine vel_cmd using given amplitude and frequency 
        g_sin_vel_cmd.data = g_amp.data * sin(radians * g_freq.data); 
        
        // publish to "vel_cmd"
        my_publisher_object.publish(g_sin_vel_cmd); 
        ROS_INFO("Commanded velocity = %f", g_sin_vel_cmd.data);

        // allow data update from callback and then wait
        ros::spinOnce();
        naptime.sleep();
    }
    // should never get here, unless roscore dies
    return 0;
}

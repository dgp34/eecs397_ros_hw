// sin_commander by dgp34
// publisher to: "vel_cmd"
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<math.h>


//global variables 
std_msgs::Float64 g_amp;
std_msgs::Float64 g_freq; 
std_msgs::Float64 g_sin_vel_cmd;

int main(int argc, char **argv) {

    // prompt user for amplitude and frequency values
    // assumed that user will input correct values......
    std::cout << "Please enter a desired amplitude.";
    std::cin >> g_amp.data;
    std::cout << "Please enter a desired frequency.";
    std::cin >> g_freq.data;


    // name node sin_commander
    ros::init(argc, argv, "sin_commander"); 
    
    // node handle
    ros::NodeHandle nh;

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

// sin_commander from p1 modified for p4_sin_two_dof by dgp34
// publisher to: "pos_cmd", "pos_cmd2"
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<math.h>


#define FREQ1 0.3
#define FREQ2 2.1
#define FREQ3 3.9


int main(int argc, char **argv) {

    // name node sin_commander_robot
    ros::init(argc, argv, "sin_commander_robot"); 
    
    // node handle
    ros::NodeHandle nh;

    // create publishers for position 
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("/three_dof_robot/joint1_position_controller/command", 1);
    ros::Publisher my_publisher_object2 = nh.advertise<std_msgs::Float64>("/three_dof_robot/joint2_position_controller/command", 1);
    ros::Publisher my_publisher_object3 = nh.advertise<std_msgs::Float64>("/three_dof_robot/joint3_position_controller/command", 1);

    // specify 10Hz controller sample rate (pretty slow, but illustrative)
    double dt_controller = 0.10;
    double sample_rate = 1.0 / dt_controller;

    // use to regulate loop rate
    ros::Rate naptime(sample_rate);

    // initialize and set everything to 0
    std_msgs::Float64 sin_pos_cmd;
    std_msgs::Float64 sin_pos_cmd2;
    std_msgs::Float64 sin_pos_cmd3;
    sin_pos_cmd.data = 0.0;
    sin_pos_cmd2.data = 0.0;
    sin_pos_cmd3.data = 0.0;
    double radians = 0.0;

    while (ros::ok()) {
        // increment radians by pi
	// radians increase by Pi/2 every second
        radians = radians + dt_controller * 3.14159265 / 2;
        
        sin_pos_cmd.data = sin(radians * FREQ1);
        sin_pos_cmd2.data = sin(radians * FREQ2);
        sin_pos_cmd3.data = sin(radians * FREQ3);
        
        // publish
        my_publisher_object.publish(sin_pos_cmd);
        my_publisher_object2.publish(sin_pos_cmd2);
        my_publisher_object3.publish(sin_pos_cmd3); 
        ROS_INFO("Commanded position 1 = %f", sin_pos_cmd.data);
        ROS_INFO("Commanded position 2 = %f", sin_pos_cmd2.data);
        ROS_INFO("Commanded position 3 = %f", sin_pos_cmd3.data);

        // allow data update from callback and then wait
        ros::spinOnce();
        naptime.sleep();
    }
    // should never get here, unless roscore dies
    return 0;
}

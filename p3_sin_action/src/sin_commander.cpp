// sin_commander action server by dgp34

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <p3_sin_action/SinActionAction.h>
#include <std_msgs/Float64.h>

#define PI 3.14159265

class SinCommander {
private:

    // we'll need a node handle; get one upon instantiation
    ros::NodeHandle nh_;

    // define the SimpleActionServer this class will contain
    actionlib::SimpleActionServer<p3_sin_action::SinActionAction> as_;
    
    // message types for client(s):  Goal, Result, and Feedback
    p3_sin_action::SinActionGoal goal_;
    p3_sin_action::SinActionResult result_;
    p3_sin_action::SinActionFeedback feedback_; // not used

    // publisher for velocity command
    ros::Publisher publisher_;



public:

    // constructor will be defined outside the class
    SinCommander();

    // destructor
    ~SinCommander(void) {    }

    // where the action happens
    void executeCB(const actionlib::SimpleActionServer<p3_sin_action::SinActionAction>::GoalConstPtr& goal);
};

// implement constructor
SinCommander::SinCommander() :
   as_(nh_, "sin_commander", boost::bind(&SinCommander::executeCB, this, _1),false) 
{
    ROS_INFO("in SinCommander constructor...");
    publisher_ = nh_.advertise<std_msgs::Float64>("vel_cmd", 1);

    as_.start();
}

// implement executeCB
void SinCommander::executeCB(const actionlib::SimpleActionServer<p3_sin_action::SinActionAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    ROS_INFO("Goal Amplitude: %f", goal->amplitude);
    ROS_INFO("Goal Frequency: %f", goal->frequency);
    ROS_INFO("Goal Cycles: %f", goal->cycles);
    
    // begin commanding

    // velocity feedback and 10Hz controller sample rate
    double Kv = 5.0;
    double dt_controller = 0.10;

    // regulate loop rate
    ros::Rate naptime(1.0 / dt_controller);

    // declare the published command
    std_msgs::Float64 vel_cmd;

    // define beginning radians and goal radians
    double radians = 0.0;
    double goalRad;

    if(goal->frequency != 0) {
       goalRad = goal->cycles * 2 * PI / goal->frequency;
    }
    else {
       goalRad = radians;
    }

    // commands occur until goal radians are reached or until ROS dies
    while (radians < goalRad && ros::ok()) {
        
        // increment radians by pi
	// radians increase by Pi/2 every second
        radians = radians + dt_controller * PI / 2;
        
        // determine vel_cmd using given amplitude and frequency 
        vel_cmd.data = goal->amplitude * sin(radians * goal->frequency); 

        // publish to "vel_cmd"
        publisher_.publish(vel_cmd); 
        ROS_INFO("Commanded velocity = %f", vel_cmd.data);
        naptime.sleep(); // wait for remainder of specified period; 
    }

    // Cycles complete.  Set velocity command to zero.
    vel_cmd.data = 0.0;
    publisher_.publish(vel_cmd);

    if(radians >= goalRad){
        result_.completed = true;
        as_.setSucceeded(result_); 
    } else {
        result_.completed = false;
        as_.setAborted(result_);
    }
}

int main(int argc, char** argv) {

    // name node sin_commander
    ros::init(argc, argv, "sin_commander");

    // action server setup
    ROS_INFO("Creating instance of sin_commander action server: ");

    SinCommander as_object;
    
    ros::spin();

    return 0;
}

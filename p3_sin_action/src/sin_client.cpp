// sin_client action client by dgp34

#include<ros/ros.h> 
#include<actionlib/client/simple_action_client.h>
#include<p3_sin_action/SinActionAction.h>

// Function called when goal reached; easy method to access "result" message
void doneCb(const actionlib::SimpleClientGoalState& state,
        const p3_sin_action::SinActionResultConstPtr& result) {
    ROS_INFO("doneCb: server responded with [%s]", state.toString().c_str());
}

int main(int argc, char** argv) {
        // initialize sin_client
        ros::init(argc, argv, "sin_client");

        // create a goal object
        p3_sin_action::SinActionGoal goal; 
        
        // link action client to action server
        actionlib::SimpleActionClient<p3_sin_action::SinActionAction> action_client("sin_commander", true);
        
        // call action server, give server "5 seconds" to respond
        // (not really...supposedly 5 seconds but actually just weird)
        ROS_INFO("Waiting for action server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(5.0));

        if (!server_exists) {
            ROS_WARN("Server not available!");
            return 0;
        }
        
        ROS_INFO("Action server has been reached!");

        while(ros::ok()) {
            // Obtain amplitude, frequency, and cycles, transfer to goal, and send goal
            std::cout << "\nWhat is your desired amplitude? ";
            std::cin >> goal.amplitude;
            std::cout << "What is your desired frequency? ";
            std::cin>> goal.frequency;
            std::cout << "For how many cycles do you want this to continue? ";
            std::cin>> goal.cycles;
            
            action_client.sendGoal(goal,&doneCb);
            
            if (!action_client.waitForResult()) {
               ROS_WARN("Taking too long...ending.");
               return 0;
            }
        }
    return 0;
}

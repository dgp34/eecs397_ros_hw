#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <xform_utils/xform_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <object_manipulation_properties/gripper_ID_codes.h>
#include <object_manipulation_properties/object_ID_codes.h>
#include <object_manipulation_properties/objectManipulationQuery.h>
#include "sticky_fingers_manip_fncs.cpp"
// to add a gripper type, add a gripper code in gripper_ID_codes.h, and
// add a case for the new gripper_id below in the switch-case block.
// Also write fnc call for this new gripper in file xxx_gripper_manip_fncs.cpp
// and include the cpp file for the gripper_manip_fncs above.

// #include "rethink_gripper_rt_manip_fncs.cpp" 
// #include "yale_gripper_model_t_manip_fncs.cpp"
// #include "ariac_vacuum_gripper_manip_fncs.cpp"

using namespace std;

bool callback(object_manipulation_properties::objectManipulationQueryRequest& request,
        object_manipulation_properties::objectManipulationQueryResponse& response) {

    // special case to test if service is alive; merely replies "valid_reply=true", but no data
    int query_code = request.query_code;
    if (query_code == object_manipulation_properties::objectManipulationQueryRequest::TEST_PING) {
        ROS_INFO("object manipulation query service received a test ping");
        response.valid_reply = true;
        return true;
    }

    // more interesting cases:
    int object_id = request.object_ID;
    int gripper_id = request.gripper_ID;

    int grasp_option = request.grasp_option;
    ROS_INFO("grasp_option = %d ", grasp_option);
    if (query_code >= object_manipulation_properties::objectManipulationQueryRequest::GET_GRASP_POSE_TRANSFORMS) {
        grasp_option = request.grasp_option;
    }

    // have to invoke this svc thrice: approach, grasp, depart poses
    // begin exhaustive cases for gripper
    switch (gripper_id) {

        case GripperIdCodes::STICKY_FINGERS:
            sticky_fingers_grasp_query(object_id, query_code, grasp_option, response);
            break;

     /* case GripperIdCodes::RETHINK_ELECTRIC_GRIPPER_RT:
            rethink_grasp_query(object_id, query_code, grasp_option, response);
            break;

        case GripperIdCodes::YALE_GRIPPER_MODEL_T:
            yale_gripper_model_t_grasp_query(object_id, query_code, grasp_option, response);
            break;
        case GripperIdCodes::ARIAC_VACUUM_GRIPPER:
            ariac_vacuum_gripper_grasp_query(object_id, query_code, grasp_option, response);
            break;
     // add more gripper cases here...
        */

        default:
            ROS_WARN("gripper ID not recognized");
            response.valid_reply = false; //just give up
            break;
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "object_manip_query_svc");
    ros::NodeHandle n;
    XformUtils xformUtils;

    ros::ServiceServer service = n.advertiseService("object_manip_query_svc", callback);
    ROS_INFO("Obj. manip. query service ready.");
    ros::spin();

    return 0;
}

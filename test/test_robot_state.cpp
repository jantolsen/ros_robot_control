// Test Robot State Node 
// -------------------------------
// Description:
//      Test Robot State Node
//
// Version:
//  0.1 - Initial Version
//        [19.12.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include Header-files:
// -------------------------------
    // Standard
    #include <memory>
    #include <iostream>
    #include <string>

    // Ros
    #include <ros/ros.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Robot-State-Processor
    #include "robot_control/robot_state_processor.h"

    // Robot Data Messages
    #include "robot_data_msgs/JointState.h"
    #include "robot_data_msgs/ToolState.h"


// Test: Function
// -------------------------------
void testFunc()
{
    
    // // Debug Print
    // // -------------------------------
    // ROS_INFO(" ");
    // ROS_INFO("Info-Kinematics:");
    // ROS_INFO("--------------------");
    // ROS_INFO_STREAM("Solver-Type: " << infoKinMsg.solver_type);
    // ROS_INFO_STREAM("Solver-Name: " << infoKinMsg.solver_name);
    // ROS_INFO_STREAM("Search-Resolution: " << infoKinMsg.search_resolution);
    // ROS_INFO_STREAM("Timeout: " << infoKinMsg.timeout);
    // ROS_INFO_STREAM("Attempts: " << infoKinMsg.attempts);
    // ROS_INFO(" ");

} // Function end: testFunc()


// Info-Kinemaatics Test Node 
// -------------------------------
int main(int argc, char** argv)
{
    // Initialization
    // -------------------------------
    // Initialize a ROS Node with a node name
    ros::init(argc, argv, "test_robot_state");   

    // Starting ROS Nodehandle(s)
    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    // Robot State Processor
    Control::RobotStateProcessor robotStateProcessor(nh);

    // Test(s)
    // -------------------------------
    // testFunc();
        
    // Shutdown
    // -------------------------------
    // ROS-Loop waiting for shutdown
    ros::waitForShutdown();

    // Function return
    return 0;
}


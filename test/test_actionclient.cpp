// Test Action-Client Node 
// -------------------------------
// Description:
//      Test Action-Client
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
    
    // ROS
    #include <ros/ros.h>

    // Action-Lib
    #include <actionlib/client/simple_action_client.h>
    #include <actionlib/client/terminal_state.h>

    // Messages
    #include "control_msgs/FollowJointTrajectoryAction.h"
    #include "sensor_msgs/JointState.h"
    #include <trajectory_msgs/JointTrajectory.h>

// Action-Client Node
// -------------------------------
int main(int argc, char** argv)
{
    // ROS Initialization
    // -------------------------------
    ros::init(argc, argv, "test_motion_client");    // Initialize ROS Node with a node name
    ros::NodeHandle nh;                             // Starting the ROS Node by declaring NodeHandle(s)

    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client("/joint_trajectory_action", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    action_client.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started");

    // Setup simple goal
    control_msgs::FollowJointTrajectoryAction motion_action;
    trajectory_msgs::JointTrajectory joint_trajectory;
    sensor_msgs::JointState joint_states;

    // Get current joint-states
    ROS_INFO("Get Current Joint-States");
    sensor_msgs::JointStateConstPtr current_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(3.0));
    if(current_state)
    {
        ROS_INFO("Update Joint-State to Current Joint-States");
        joint_states = *current_state;
        if(joint_states.position.empty())
        {
            ROS_ERROR_STREAM("Failed: current joint state is empty.");
        }

        ROS_INFO("Get Joint-State Names");
        for (std::size_t i = 0 ; i < joint_states.name.size() ; i++)
        {
            // Print name in terminal
            ROS_INFO("Joint Name: %s", joint_states.name[i].c_str());
            joint_trajectory.joint_names.push_back(joint_states.name[i]);
        }
    }

    // Create Trajectory Point
    ROS_INFO("Creating Trajectory Points:");
    trajectory_msgs::JointTrajectoryPoint p0;
    trajectory_msgs::JointTrajectoryPoint p1;
    trajectory_msgs::JointTrajectoryPoint p2;

    // POINT 0:
    ROS_INFO("Point 0:");
    // Set the start of the trajectory equal to the current joint-states
    p0.positions = joint_states.position;
    p0.time_from_start = ros::Duration(0.0);

    // POINT 1:
    ROS_INFO("Point 1:");
    p1.positions = joint_states.position;
    p1.positions[3] = 20 * (M_PI / 180);
    p1.positions[5] = 20 * (M_PI / 180);
    p1.time_from_start = ros::Duration(5.0);

    // POINT 2:
    ROS_INFO("Point 2:");
    p2.positions = joint_states.position;
    p1.positions[3] = -20 * (M_PI / 180);
    p2.positions[5] = -320 * (M_PI / 180);
    p2.time_from_start = ros::Duration(6.0);

    // Create Trajectory
    ROS_INFO("Creating Trajectory");
    joint_trajectory.points.clear();
    
    // add points
    ROS_INFO("Adding Point to Trajectory");
    // joint_trajectory.points.front() = p0;
    joint_trajectory.points.push_back(p0);
    joint_trajectory.points.push_back(p1);
    joint_trajectory.points.push_back(p2);
    

    // Add trajectory to goal
    ROS_INFO("Adding Trajectory to Action-Goal");
    motion_action.action_goal.goal.trajectory = joint_trajectory;

    // Submit goal for execution
    ROS_INFO("Sending Goal");
    action_client.sendGoal(motion_action.action_goal.goal);
    ROS_INFO("Executing Trajectory");

    ROS_INFO("Waiting for completion");
    action_client.waitForResult();
    ROS_INFO("Done");
}
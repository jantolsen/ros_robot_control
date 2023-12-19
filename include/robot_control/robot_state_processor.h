// Robot State Processor
// -------------------------------
// Description:
//      Robot State Processor is responsible for collection state information on the robotic system
//      and extending the state information based on calculations on existing state data.
//      This tycially involves publishing calculated joint-state velocities and acceleration,
//      based on joint-state positional readings.
//
// Version:
//  0.1 - Initial Version
//        [18.12.2023]  -   Jan T. Olsen
//
// -------------------------------

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef ROBOT_STATE_PROCESSOR_H       
#define ROBOT_STATE_PROCESSOR_H

// Include Header-files:
// -------------------------------
    // Standard
    #include <iostream>
    #include <string>
    #include <vector>

    // Ros
    #include <ros/ros.h>

    // ROS Messages
    #include <sensor_msgs/JointState.h>
    #include <geometry_msgs/Twist.h>

    // Robotics Toolbox
    #include "robot_toolbox/toolbox.h"

    // Robotics Messages
    #include "robot_data_msgs/JointState.h"
    #include "robot_data_msgs/ToolState.h"

// Namespace: Control
// -------------------------------
namespace Control
{

// Robot State Processor
// -------------------------------
/** \brief Robot State Processor
*
* Responsible for collection state information on the robotic system
* and extending the state information based on calculations on existing state data.
* This involves calculating missing information on joint-state velocities and accelerations
* using the available reading of joint-state position
*/
class RobotStateProcessor
{
    // Public Class members
    // -------------------------------
    // Accessible for everyone
    public:

        // Class constructor
        // -------------------------------
        /** \brief Robot State Processor Constuctor
        * \param nh     ROS Nodehandle
        */
        RobotStateProcessor(
            ros::NodeHandle& nh);


        // Class destructor
        // -------------------------------
        /** \brief Robot State Processor destructor
        */
        ~RobotStateProcessor();


    // Protected Class members
    // -------------------------------
    // Accessible within the class which defines them, 
    // and classes which inherits from the parent class
    protected:

        // Initialize Robot State Processor
        // -------------------------------
        /** \brief Initialize Robot State Processor
        */
        void init();


        // Joint-State Callback
        // -------------------------------
        /** \brief Joint-State Callback Function
        *
        * Callback function for the Joint-State subscriber.
        * Function is registered to be called whenever a new message is received 
        * on the joint-state topic. The function then processes the received joint-state 
        * message and calculates an extend joint-state information (velocity and acceleration).
        *
        * \param joint_state_msg    Received Joint-State message pointer [sensor_msgs::JointState::ConstPtr&]
        */
        void jointStateCallback(
            const sensor_msgs::JointState::ConstPtr& joint_state_msg);


        // Tool-State Callback
        // -------------------------------
        /** \brief Tool-State Callback
        *
        * This is not a direct callback function for Tool-State, but is rather registered
        * to be called whenever the joint-state-callback function is called/executed. 
        * The function listens for transformation using the transformation listener, 
        * and then processes the obtained tool-state to calculate an extend tool-state 
        * containing tool velocity and and acceleration information.
        */
        void toolStateCallback();


        // Calculate Joint-State Derivatives
        // -------------------------------
        /** \brief Calculate Joint-State Derivatives
        *
        * Calculates joint-state velocity and acceleration using incomming position readings and the time-stamp.
        *
        * \param joint_state_msg    Received Joint-State message pointer [sensor_msgs::JointState::ConstPtr&]
        * \param joint_state_calc   Calculated Joint-State message with velocity and acceleration [robot_data_msgs::JointState]
        */
        void calculateJointDerivatives(
            const sensor_msgs::JointState::ConstPtr& joint_state_msg,
            robot_data_msgs::JointState& joint_state_calc);


        // Calculate Tool-State Derivatives
        // -------------------------------
        /** \brief Calculate Tool-State Derivatives
        *
        * Calculates tool-state velocity and acceleration using incomming position readings and the time-stamp.
        *
        * \param tool_transform     Received Tool-State transformation [geometry_msgs::TransformStamped]
        * \param tool_state_calc    Calculated Tool-State with velocity and acceleration [robot_data_msgs::ToolState]
        */
        void calculateToolDerivatives(
            const geometry_msgs::TransformStamped& tool_transform,
            robot_data_msgs::ToolState& tool_state_calc);


    // Private Class members
    // -------------------------------
    // Accessible only for the class which defines them
    private:
        // Class-Name-Prefix for terminal message
        static const std::string CLASS_PREFIX;

        // ROS Nodehandle(s)
        // -------------------------------
        ros::NodeHandle nh_;

        // Transformation manager
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        // Class Local Member(s)
        // -------------------------------
        ros::Time prev_joint_timestamp_;
        ros::Time prev_tool_timestamp_;
        std::vector<double> prev_joint_position_;
        std::vector<double> prev_joint_velocity_;
        robot_toolbox::PoseRPY prev_tool_pose_;
        geometry_msgs::Twist prev_tool_velocity_;
        
        // ROS Subscriber(s)
        // -------------------------------
        ros::Subscriber controller_joint_state_sub_;

        // ROS Publsiher(s)
        // -------------------------------
        ros::Publisher joint_state_pub_;
        ros::Publisher tool_state_pub_;


}; // End Class: RobotStateProcessor
} // End Namespace: Control
#endif // ROBOT_STATE_PROCESSOR_H 
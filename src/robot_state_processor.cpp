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

// Include Header-files:
// -------------------------------
    
    // Class Header-File
    #include "robot_control/robot_state_processor.h"

// Namespace: Control
// -------------------------------
namespace Control
{
    // RobotStateProcessor Class
    // -------------------------------

        // Constants
        // -------------------------------
        const std::string RobotStateProcessor::CLASS_PREFIX = "RobotStateProcessor::";


        // Class constructor
        // -------------------------------
        // (Constructor Overloading)
        RobotStateProcessor::RobotStateProcessor(
            ros::NodeHandle& nh)
        :
            nh_(nh)
        {
            // Initialize Template-Class
            init();
        } // Class Constructor End: RobotStateProcessor()


        // Class Desctructor
        // -------------------------------
        RobotStateProcessor::~RobotStateProcessor()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Destructor called");

        } // Class Desctructor End: ~RobotStateProcessor()


        // Initialize RobotStateProcessor
        // -------------------------------
        void RobotStateProcessor::init()
        {
            // Report to terminal
            ROS_INFO_STREAM(CLASS_PREFIX << __FUNCTION__ 
                << ": Initializing class");
        } // Function End: init()


        // Joint-State Callback
        // -------------------------------
        void RobotStateProcessor::jointStateCallback(
            const sensor_msgs::JointState::ConstPtr& joint_state_msg)
        {

        } // Function End: jointStateCallback()


        // Calculate Derivatives
        // -------------------------------
        void RobotStateProcessor::calculateDerivatives(
            const sensor_msgs::JointState::ConstPtr& joint_state_msg,
            sensor_msgs::JointState& joint_state_calc)
        {
            // Calculate time difference
            ros::Duration dt = joint_state_msg->header.stamp - prev_timestamp_;

        } // Function End: calculateDerivatives()


} // End Namespace: Control
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
            nh_(nh),
            tfListener_(tfBuffer_)
        {
            // Initialize Subscriber
            controller_joint_state_sub_ = nh_.subscribe("/controller_joint_states", 1, &RobotStateProcessor::jointStateCallback, this);

            // Initialize Publisher
            joint_state_pub_ = nh_.advertise<robot_data_msgs::JointState>("robot_data/joint_states", 1);
            tool_state_pub_ = nh_.advertise<robot_data_msgs::ToolState>("robot_data/tool_states", 1);

            // Transformation Manager
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener(tfBuffer);

            // Initialize RobotStateProcessor
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
            // Check for empty previous position or zero previous timestamp
            if(prev_joint_position_.empty() || (prev_joint_timestamp_ == ros::Time(0)))
            {
                // Initialize previous joint state
                prev_joint_position_.resize(joint_state_msg->name.size(), 0.0);
                prev_joint_velocity_.resize(joint_state_msg->name.size(), 0.0);

                // Iterate through joint-state message
                for (int i = 0; i < joint_state_msg->name.size(); i++)
                {
                    // Assign previous joint position to joint-state position
                    prev_joint_position_[i] = joint_state_msg->position[i];
                    // Assign previous joint velocity to zero 
                    prev_joint_velocity_[i] = 0.0;
                }

                // Assign previous timestamp to current timestamp
                prev_joint_timestamp_ = joint_state_msg->header.stamp;

                // Function return
                return;
            }

            // Create new Joint-State
            robot_data_msgs::JointState joint_state_calc_;

            // Populate header, name and position fields
            joint_state_calc_.header = joint_state_msg->header;
            joint_state_calc_.name = joint_state_msg->name;
            joint_state_calc_.position = joint_state_msg->position;

            // Calculate Derivatives (Velocity and Acceleration)
            calculateJointDerivatives(joint_state_msg, joint_state_calc_);

            // Publish Joint-State
            joint_state_pub_.publish(joint_state_calc_);

            // Tool-State Callback
            toolStateCallback();
        } // Function End: jointStateCallback()


        // Tool-State Callback
        // -------------------------------
        void RobotStateProcessor::toolStateCallback()
        {
            // Get transformation from world to tool
            geometry_msgs::TransformStamped tool_transform_stamped; 
            Toolbox::Kinematics::getCurrentTransform("tool0", "world", tool_transform_stamped);

            // Create new Tool-State
            robot_data_msgs::ToolState tool_state_calc_;

            // Populate header, name and pose fields
            tool_state_calc_.header = tool_transform_stamped.header;
            tool_state_calc_.frame = tool_transform_stamped.header.frame_id;
            tool_state_calc_.name = tool_transform_stamped.child_frame_id;

            // Convert transform to pose
            geometry_msgs::Pose tool_pose = Toolbox::Convert::transformToPose(tool_transform_stamped.transform);

            // Convert pose to pose-rpy
            tool_state_calc_.pose = Toolbox::Convert::poseToPoseRPY(tool_pose);

            // Calculate Derivatives (Velocity and Acceleration)
            calculateToolDerivatives(tool_transform_stamped, tool_state_calc_);

            // Publish Tool-State
            tool_state_pub_.publish(tool_state_calc_);
        }


        // Calculate Joint-State Derivatives
        // -------------------------------
        void RobotStateProcessor::calculateJointDerivatives(
            const sensor_msgs::JointState::ConstPtr& joint_state_msg,
            robot_data_msgs::JointState& joint_state_calc)
        {
            // Calculate time difference
            ros::Duration dt = joint_state_msg->header.stamp - prev_joint_timestamp_;

            // Check for zero time difference
            if(dt.toSec() == 0.0)
            {
                // Function return
                return;
            }

            // Iterate through joint-state message
            for (int i = 0; i < joint_state_msg->name.size(); i++)
            {
                // Calculate joint velocity
                // (numerical differentiation)
                double velocity = Toolbox::Math::numericalDifferentiation(joint_state_msg->position[i], prev_joint_position_[i], dt.toSec());
                joint_state_calc.velocity.push_back(velocity);

                // Calculate joint acceleration
                // (numerical differentiation)
                double acceleration = Toolbox::Math::numericalDifferentiation(velocity, prev_joint_velocity_[i], dt.toSec());
                joint_state_calc.acceleration.push_back(acceleration);

                // Update previous joint values
                prev_joint_position_[i] = joint_state_msg->position[i];
                prev_joint_velocity_[i] = velocity;
            }

            // Update previous timestamp
            prev_joint_timestamp_ = joint_state_msg->header.stamp;
        } // Function End: calculateJointDerivatives()


        // Calculate Tool-State Derivatives
        // -------------------------------
        void RobotStateProcessor::calculateToolDerivatives(
            const geometry_msgs::TransformStamped& tool_transform_stamped,
            robot_data_msgs::ToolState& tool_state_calc)
        {
            // Calculate time difference
            ros::Duration dt = tool_transform_stamped.header.stamp - prev_tool_timestamp_;

            // Check for zero time difference
            if(dt.toSec() == 0.0)
            {
                // Function return
                return;
            }

            // Calculate tool translational velocity
            // (numerical differentiation)
            tool_state_calc.velocity.linear.x = Toolbox::Math::numericalDifferentiation(tool_transform_stamped.transform.translation.x, prev_tool_pose_.position.x, dt.toSec());
            tool_state_calc.velocity.linear.y = Toolbox::Math::numericalDifferentiation(tool_transform_stamped.transform.translation.y, prev_tool_pose_.position.y, dt.toSec());
            tool_state_calc.velocity.linear.z = Toolbox::Math::numericalDifferentiation(tool_transform_stamped.transform.translation.z, prev_tool_pose_.position.z, dt.toSec());

            // Calculate tool rotational velocity
            // (numerical differentiation)
            geometry_msgs::Point euler_angles = Toolbox::Convert::quaternionToEuler(tool_transform_stamped.transform.rotation);
            tool_state_calc.velocity.angular.x = Toolbox::Math::numericalDifferentiation(Toolbox::Convert::radToDeg(euler_angles.x), prev_tool_pose_.orientation.x, dt.toSec());
            tool_state_calc.velocity.angular.y = Toolbox::Math::numericalDifferentiation(Toolbox::Convert::radToDeg(euler_angles.y), prev_tool_pose_.orientation.y, dt.toSec());
            tool_state_calc.velocity.angular.z = Toolbox::Math::numericalDifferentiation(Toolbox::Convert::radToDeg(euler_angles.z), prev_tool_pose_.orientation.z, dt.toSec());

            // Calculate tool translational acceleration
            // (numerical differentiation)
            tool_state_calc.acceleration.linear.x = Toolbox::Math::numericalDifferentiation(tool_state_calc.velocity.linear.x, prev_tool_velocity_.linear.x, dt.toSec());
            tool_state_calc.acceleration.linear.y = Toolbox::Math::numericalDifferentiation(tool_state_calc.velocity.linear.y, prev_tool_velocity_.linear.y, dt.toSec());
            tool_state_calc.acceleration.linear.z = Toolbox::Math::numericalDifferentiation(tool_state_calc.velocity.linear.z, prev_tool_velocity_.linear.z, dt.toSec());

            // Calculate tool rotational acceleration
            // (numerical differentiation)
            tool_state_calc.acceleration.angular.x = Toolbox::Math::numericalDifferentiation(tool_state_calc.velocity.angular.x, prev_tool_velocity_.angular.x, dt.toSec());
            tool_state_calc.acceleration.angular.y = Toolbox::Math::numericalDifferentiation(tool_state_calc.velocity.angular.y, prev_tool_velocity_.angular.y, dt.toSec());
            tool_state_calc.acceleration.angular.z = Toolbox::Math::numericalDifferentiation(tool_state_calc.velocity.angular.z, prev_tool_velocity_.angular.z, dt.toSec());

            // Update previous tool values
            prev_tool_pose_ = tool_state_calc.pose;
            prev_tool_velocity_ = tool_state_calc.velocity;

            // Update previous timestamp
            prev_tool_timestamp_ = tool_transform_stamped.header.stamp;
        } // Function End: calculateToolDerivatives()

} // End Namespace: Control
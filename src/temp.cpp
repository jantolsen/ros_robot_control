#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class JointStateProcessor {
public:
    JointStateProcessor() {
        // Subscribe to the /joint_states topic
        joint_states_sub_ = nh_.subscribe("/joint_states", 10, &JointStateProcessor::jointStatesCallback, this);

        // Publisher for the joint state message with velocity and acceleration
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states_with_derivatives", 10);
    }

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& joint_states_msg) {
        // Check if we have previous joint positions and timestamps
        if (prev_positions_.find(joint_states_msg->name[0]) == prev_positions_.end() ||
            prev_timestamp_ == ros::Time(0)) {
            // If not, initialize previous positions and timestamp with current values
            for (size_t i = 0; i < joint_states_msg->name.size(); ++i) {
                prev_positions_[joint_states_msg->name[i]] = joint_states_msg->position[i];
            }
            prev_timestamp_ = joint_states_msg->header.stamp;
            return;
        }

        // Create a new JointState message
        sensor_msgs::JointState joint_state_with_derivatives_msg;

        // Populate the header
        joint_state_with_derivatives_msg.header = joint_states_msg->header;

        // Populate the name and position fields
        joint_state_with_derivatives_msg.name = joint_states_msg->name;
        joint_state_with_derivatives_msg.position = joint_states_msg->position;

        // Calculate joint velocities and accelerations
        calculateDerivatives(joint_states_msg, joint_state_with_derivatives_msg);

        // Publish joint state message with derivatives
        joint_state_pub_.publish(joint_state_with_derivatives_msg);
    }

private:
    void calculateDerivatives(const sensor_msgs::JointState::ConstPtr& joint_states_msg,
                               sensor_msgs::JointState& joint_state_with_derivatives_msg) {
        // Assuming joint_states_msg contains position data only

        // Calculate time difference
        ros::Duration dt = joint_states_msg->header.stamp - prev_timestamp_;

        // Populate the velocity and acceleration fields
        for (size_t i = 0; i < joint_states_msg->name.size(); ++i) {
            // Calculate joint velocities (difference / time)
            double velocity = (joint_states_msg->position[i] - prev_positions_[joint_states_msg->name[i]]) / dt.toSec();
            joint_state_with_derivatives_msg.velocity.push_back(velocity);

            // Calculate joint accelerations (difference / time)
            double acceleration = (velocity - prev_velocities_[joint_states_msg->name[i]]) / dt.toSec();
            joint_state_with_derivatives_msg.effort.push_back(acceleration);

            // Update previous values
            prev_positions_[joint_states_msg->name[i]] = joint_states_msg->position[i];
            prev_velocities_[joint_states_msg->name[i]] = velocity;
        }

        // Update timestamp
        prev_timestamp_ = joint_states_msg->header.stamp;
    }

    ros::NodeHandle nh_;
    ros::Subscriber joint_states_sub_;
    ros::Publisher joint_state_pub_;

    std::map<std::string, double> prev_positions_;
    std::map<std::string, double> prev_velocities_;
    ros::Time prev_timestamp_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_state_processor_node");

    JointStateProcessor joint_state_processor;

    ros::spin();

    return 0;
}

#ifndef PROJ1_SETUP_NODE_HPP
#define PROJ1_SETUP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/navigator_states.hpp>
#include <baxter_core_msgs/msg/navigator_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/time.h>

namespace ECNBaxter {
    using baxter_core_msgs::msg::NavigatorStates;
    using baxter_core_msgs::msg::NavigatorState;
    using geometry_msgs::msg::TransformStamped;
    using geometry_msgs::msg::Pose;

enum class SetupStep {
    IDLE,
    LEFT_SOURCE,
    LEFT_DESTINATION,
    RIGHT_SOURCE,
    RIGHT_DESTINATION,
    ENDED
};

class SetupNode : public rclcpp::Node {
    public:
        SetupNode(rclcpp::NodeOptions &opts);
    protected:
        SetupStep current_step = SetupStep::IDLE;
        bool pushed = false;
        rclcpp::Subscription<NavigatorStates>::SharedPtr states_sub;
        tf2_ros::Buffer tf_buffer;

        std::map<SetupStep, Pose> saved_position;

        void increment_step();
        void navigator_callback(NavigatorStates::SharedPtr &states);
        void savePose();
    };
}

#endif //PROJ1_SETUP_NODE_HPP

#include <ecn_baxter/game/setup_node.hpp>

namespace ECNBaxter {
    SetupNode::SetupNode(rclcpp::NodeOptions &opts) : rclcpp::Node("setup_node", opts) {
        states_sub = create_subscription<NavigatorStates>(
                "/robot/navigators_states",
                1,
                [this] (NavigatorStates::SharedPtr msg) {
                    navigator_callback(msg);
                });
    }

    void SetupNode::increment_step() {
        switch (current_step) {
            case SetupStep::IDLE:
                current_step = SetupStep::LEFT_SOURCE;
                break;
            case SetupStep::LEFT_SOURCE:
                current_step = SetupStep::LEFT_DESTINATION;
                break;
            case SetupStep::LEFT_DESTINATION:
                current_step = SetupStep::RIGHT_SOURCE;
                break;
            case SetupStep::RIGHT_SOURCE:
                current_step = SetupStep::RIGHT_DESTINATION;
                break;
            case SetupStep::RIGHT_DESTINATION:
                current_step = SetupStep::ENDED;
                break;
            case SetupStep::ENDED:
                break;
        }
    }

    void SetupNode::navigator_callback(NavigatorStates::SharedPtr &states) {
        if (pushed) {
            for (auto state : states->states) {
                if (state.buttons[0]) {
                    return;
                }
                pushed = false;
            }
        }
        else {
            for (auto state : states->states) {
                if (state.buttons[0]) {
                    increment_step();
                    pushed = true;
                    return;
                }
            }
        }
    }

    void SetupNode::savePose() {
        std::string side = "none";
        switch (current_step) {
            case SetupStep::LEFT_SOURCE:
            case SetupStep::LEFT_DESTINATION:
                side = "left_gripper";
                break;
            case SetupStep::RIGHT_SOURCE:
            case SetupStep::RIGHT_DESTINATION:
                side = "right_gripper";
                break;
        }
        const std::string base = "base";
        if (!tf_buffer.canTransform(side, base, tf2::TimePointZero, tf2::durationFromSec(1.0))) {
            RCLCPP_FATAL(get_logger(), "Can't find corresponding frame");
            return;
        }
        
    }
}
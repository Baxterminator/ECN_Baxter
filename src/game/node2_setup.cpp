#include <ecn_baxter/game/node2.hpp>

namespace ECNBaxter {
    void GMROS2::launch_setup() {
        using namespace std::placeholders;
        RCLCPP_INFO(get_logger(), "Setup launch !");

        // Setup MSG
        auto setup_msg = PointsSetup::Goal();
        setup_msg.ptns_name = std::vector<std::string>{
            "test1",
            "test2",
            "test3",
            "test4"
        };
        setup_msg.sides = std::vector<bool>{
          setup_msg.LEFT,
          setup_msg.LEFT,
          setup_msg.RIGHT,
          setup_msg.RIGHT
        };

        auto goal_options = Client<PointsSetup>::SendGoalOptions();
        goal_options.goal_response_callback = [&](const PtnSetupHandler::SharedPtr &handle) {
            ptn_setup_goal(handle);
        };
        goal_options.feedback_callback = [&] (PtnSetupHandler::SharedPtr handle, const sptr<const PointsSetup::Feedback> feedback) {
            ptn_setup_feedback(handle, feedback);
        };
        goal_options.result_callback = [&] (const PtnSetupHandler::WrappedResult &result) {
            ptn_setup_result(result);
        };

        ptn_setup->async_send_goal(setup_msg, goal_options);
    }
}
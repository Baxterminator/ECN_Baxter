#include <ecn_baxter/game/node2.hpp>

namespace ECNBaxter {

    GMROS2::GMROS2(rclcpp::NodeOptions opts) : rclcpp::Node("game_master_2", opts) {
        ptn_setup = rclcpp_action::create_client<PointsSetup>(this, "points_setup");
    }

    void GMROS2::bind_gui() {
        QObject::connect(UIWrapper::setup(), &QPushButton::clicked, [this]() {launch_setup();});
    }

    /* =================================================================
     #                              ACTION
     #                            Points setup
     # =================================================================*/

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
        goal_options.goal_response_callback = [&](std::shared_future<PtnSetupHandler::SharedPtr> future) {
            ptn_setup_goal(future);
        };
        goal_options.goal_response_callback = std::bind(&GMROS2::ptn_setup_goal, this);
        goal_options.feedback_callback = [&] (PtnSetupHandler::SharedPtr handle, const sptr<const PointsSetup::Feedback> feedback) {
            ptn_setup_feedback(handle, feedback);
        };
        goal_options.result_callback = [&] (const PtnSetupHandler::WrappedResult &result) {
            ptn_setup_result(result);
        };

        ptn_setup->async_send_goal(setup_msg, goal_options);
    }

    void GMROS2::ptn_setup_goal(const PtnSetupHandler::SharedPtr &handle) {
        if (!handle) {
            RCLCPP_ERROR(get_logger(), "Setup rejected bu server");
        }
        RCLCPP_INFO(get_logger(), "Setup launched !");
    }

    void GMROS2::ptn_setup_goal(std::shared_future<PtnSetupHandler::SharedPtr> future) {
        auto handle = future.get();
        ptn_setup_goal(handle);
    }

    void GMROS2::ptn_setup_result(const PtnSetupHandler::WrappedResult &result) {
        switch (result.code) {
            case ResultCode::ABORTED:
                RCLCPP_ERROR(get_logger(), "Setup aborted !");
                break;
            case ResultCode::CANCELED:
                RCLCPP_ERROR(get_logger(), "Setup canceled !");
                break;
            case ResultCode::UNKNOWN:
                RCLCPP_ERROR(get_logger(), "Unknown result code :/");
                break;
        }
        RCLCPP_INFO(get_logger(), "Setup done with result %d", result.result->success);
    }

    void GMROS2::ptn_setup_feedback(PtnSetupHandler::SharedPtr &handle,
                                    const sptr<const PointsSetup::Feedback> feedback) 
    {
        auto p_name = feedback->ptn_name;
        auto p = feedback->ptn;
        RCLCPP_INFO(get_logger(),
        "Receiving marker %s at (%f, %f, %f)",
        p_name.c_str(),
        p.x,
        p.y,
        p.z);
    }
}
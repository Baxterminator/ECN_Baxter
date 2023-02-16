#include <ecn_baxter/game/node2.hpp>
#include <iostream>

namespace ECNBaxter {

    GMROS2::GMROS2(rclcpp::NodeOptions opts) : rclcpp::Node("game_master_2", opts) {
        ptn_setup = rclcpp_action::create_client<PointsSetup>(this, "points_setup");
    }

    void GMROS2::bind_gui() {
        UIWrapper main_gui;
        QObject::connect(main_gui.setup(), &QPushButton::clicked, [this]() {launch_setup();});
        QObject::connect(main_gui.load_game(), &QPushButton::clicked, [this]() {
            FileLoaderWrapper::display();
        });
    }

    /* =================================================================
     #                              ACTION
     #                            Points setup
     # =================================================================*/
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
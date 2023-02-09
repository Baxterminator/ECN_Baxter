#ifndef NODE2_HPP
#define NODE2_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ecn_baxter/game/gui_wrapper.hpp>
#include <ecn_baxter/action/points_setup.hpp>

#include <future>
#include <memory>
#include <string>
#include <sstream>

namespace ECNBaxter {

    using rclcpp_action::Client;
    using rclcpp_action::Client;
    using rclcpp_action::ResultCode;
    using ecn_baxter::action::PointsSetup;
    using PtnSetupHandler = rclcpp_action::ClientGoalHandle<PointsSetup>;

    template<typename F>
    using sptr = std::shared_ptr<F>;

    template<class T>
    using legacy_goal_callback = std::function<void(T)>;

/**
 * ROS 2 Node for the game master
*/
class GMROS2 : public rclcpp::Node {
public:
    explicit GMROS2(rclcpp::NodeOptions opts);
    void bind_gui();
protected:
    /* =================================================================
     #                              ACTION
     #                            Points setup
     # =================================================================*/
    Client<PointsSetup>::SharedPtr ptn_setup;
    void launch_setup();
    void ptn_setup_goal(const PtnSetupHandler::SharedPtr &handle);
    void ptn_setup_goal(std::shared_future<PtnSetupHandler::SharedPtr> future);
    void ptn_setup_feedback(PtnSetupHandler::SharedPtr &handle,
                            const sptr<const PointsSetup::Feedback> feedback);
    void ptn_setup_result(const PtnSetupHandler::WrappedResult &result);
};

}
#endif
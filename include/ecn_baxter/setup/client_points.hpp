/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS 2 Node part for point setuping
 *========================================================================**/

#ifndef SETUP_PTN_CLIENT_HPP
#define SETUP_PTN_CLIENT_HPP

#include <ecn_baxter/action/points_setup.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ecn_baxter::setup
{
    using ecn_baxter::action::PointsSetup;
    using rclcpp_action::Client;
    using rclcpp_action::ResultCode;
    using PtnSetupHandler = rclcpp_action::ClientGoalHandle<PointsSetup>;
    template <typename F>
    using sptr = std::shared_ptr<F>;

    /// @brief ROS 2 Node part for point setuping

    class SetupPointsClient
    {
    public:
        explicit SetupPointsClient(rclcpp::Logger ros2_logger)
            : logger(ros2_logger) {}

    protected:
        rclcpp::Logger logger;
        Client<PointsSetup>::SharedPtr ptn_setup;
        void launch_setup();

        /// @brief Callback of the acceptance of the action call
        /// @param future the future linked to the call of the action
        /// @deprecated This function is only for legacy ROS2 version support (foxy, galactic, ...).
        /// Use PtnSetupHandler::SharedPtr from humble and onward
        inline void ptn_setup_goal(std::shared_future<PtnSetupHandler::SharedPtr> future)
        {
            auto handle = future.get();
            ptn_setup_goal(handle);
        }
        void ptn_setup_goal(const PtnSetupHandler::SharedPtr &handle);
        void ptn_setup_feedback(PtnSetupHandler::SharedPtr &handle,
                                const sptr<const PointsSetup::Feedback> feedback);
        void ptn_setup_result(const PtnSetupHandler::WrappedResult &result);
    };

}
#endif
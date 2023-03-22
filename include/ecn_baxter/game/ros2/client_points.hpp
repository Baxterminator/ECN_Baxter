/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS 2 Node part for point setuping
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_SETUP_PTN_CLIENT_HPP
#define ECN_BAXTER_SETUP_PTN_CLIENT_HPP

#include "ecn_baxter/action/detail/points_setup__struct.hpp"
#include "ecn_baxter/action/points_setup.hpp"
#include "ecn_baxter/base/base_action_client.hpp"
#include "ecn_baxter/game/data/game_properties.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ecn_baxter::game::ros2 {
using ecn_baxter::action::PointsSetup;
using PtnSetupHandler = rclcpp_action::ClientGoalHandle<PointsSetup>;

/// @brief ROS 2 Node part for point setuping
class SetupPointsClient : public ecn::base::BaseActionClient<PointsSetup> {
public:
  explicit SetupPointsClient(std::shared_ptr<rclcpp::Node> node)
      : BaseActionClient(node, "point_server"), logger(node->get_logger()) {}

  void update_ptn_setup_client(std::weak_ptr<data::GameProperties> gprop) {
    game_props = gprop;
  }

  void make_action_call() override;

protected:
  rclcpp::Logger logger;

  void handle_goal(const PtnSetupHandler::SharedPtr) override;
  void
  handle_feedback(PtnSetupHandler::SharedPtr,
                  const std::shared_ptr<const PointsSetup::Feedback>) override;
  void handle_result(const PtnSetupHandler::WrappedResult) override;

private:
  std::weak_ptr<data::GameProperties> game_props;
};

} // namespace ecn_baxter::game::ros2
#endif
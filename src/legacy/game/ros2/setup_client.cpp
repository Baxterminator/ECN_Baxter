/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS 2 Node part for point setuping
 * @deprecated     :  These functions are only for legacy ROS2 version support
 *                    (foxy, galactic, ...).
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#include <ecn_baxter/game/ros2/client_points.hpp>

namespace ecn_baxter::game::ros2 {
/// @brief Launch the call to the action for point setuping
/// @deprecated This function is only for legacy ROS2 version support (foxy,
/// galactic, ...).
void SetupPointsClient::launch_point_setup() {
  using namespace std::placeholders;

  // Verifying that the game props are still existent
  if (game_props.expired()) {
    RCLCPP_WARN(logger, "Game Properties ptr is expired !");
    return;
  }

  auto props = game_props.lock();

  RCLCPP_INFO(logger, "Launching setup phase !");

  // Setup MSG
  auto setup_msg = PointsSetup::Goal();
  setup_msg.ptns_name = std::vector<std::string>(0);
  setup_msg.sides = std::vector<bool>(0);

  RCLCPP_INFO(logger, "Preparing %zu markers to setup",
              props->setup.needed_points.size());
  for (auto point : props->setup.needed_points) {
    setup_msg.ptns_name.push_back(point.name);
    setup_msg.sides.push_back(game::data::side2bool(point.arm_side));
  }

  auto goal_options = Client<PointsSetup>::SendGoalOptions();
  goal_options.goal_response_callback =
      [&](std::shared_future<PtnSetupHandler::SharedPtr> future) {
        ptn_setup_goal(future);
      };
  goal_options.feedback_callback =
      [&](PtnSetupHandler::SharedPtr handle,
          const sptr<const PointsSetup::Feedback> feedback) {
        ptn_setup_feedback(handle, feedback);
      };
  goal_options.result_callback =
      [&](const PtnSetupHandler::WrappedResult &result) {
        ptn_setup_result(result);
      };

  RCLCPP_INFO(logger, "Calling action server");
  ptn_setup->async_send_goal(setup_msg, goal_options);
}
} // namespace ecn_baxter::game::ros2
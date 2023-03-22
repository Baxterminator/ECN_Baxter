/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS 2 Node part for point setuping
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/ros2/client_points.hpp"
#include "ecn_baxter/action/detail/points_setup__struct.hpp"
#include <algorithm>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ecn_baxter::game::ros2 {
/// @brief Callback of the acceptance of the action call
/// @param handle the handle to the call to the action
void SetupPointsClient::handle_goal(const PtnSetupHandler::SharedPtr handle) {
  if (!handle) {
    RCLCPP_ERROR(logger, "Setup rejected bu server");
  }
  RCLCPP_INFO(logger, "Setup launched !");
}

/// @brief Callback of the last message sent by the action
/// @param result the end result of the action call
void SetupPointsClient::handle_result(
    const PtnSetupHandler::WrappedResult result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(logger, "Setup aborted !");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(logger, "Setup canceled !");
    break;
  case rclcpp_action::ResultCode::UNKNOWN:
    RCLCPP_ERROR(logger, "Unknown result code :/");
    break;
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  }
  RCLCPP_INFO(logger, "Setup done with result %d", result.result->success);
}

/// @brief Callback of the feedback sent by the setuping action
/// @param handle the handle to the call to the action
/// @param feedback the position and name of the next POI
void SetupPointsClient::handle_feedback(
    [[maybe_unused]] PtnSetupHandler::SharedPtr handle,
    const std::shared_ptr<const PointsSetup::Feedback> feedback) {
  auto p_name = feedback->ptn_name;
  auto p = feedback->ptn;
  RCLCPP_INFO(logger, "Receiving marker %s at (%f, %f, %f)", p_name.c_str(),
              p.x, p.y, p.z);

  // Saving it to the game properties
  if (!game_props.expired()) {
    auto s_props = game_props.lock();

    geometry_msgs::TransformStamped ptn_transform;
    ptn_transform.child_frame_id = feedback->ptn_name;
    ptn_transform.transform.translation.x = feedback->ptn.x;
    ptn_transform.transform.translation.y = feedback->ptn.y;
    ptn_transform.transform.translation.z = feedback->ptn.z;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    ptn_transform.transform.rotation.w = q.w();
    ptn_transform.transform.rotation.x = q.x();
    ptn_transform.transform.rotation.y = q.y();
    ptn_transform.transform.rotation.z = q.z();
    ptn_transform.header.frame_id = "base";

    s_props->setup.ptn_msgs.transforms.push_back(ptn_transform);
  }
}

/// @brief Launch the call to the action for point setuping
void SetupPointsClient::make_action_call() {
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
  launch_action(setup_msg);
}

} // namespace ecn_baxter::game::ros2

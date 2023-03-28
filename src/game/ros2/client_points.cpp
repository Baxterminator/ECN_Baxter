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
#include "ecn_baxter/game/events/event_target.hpp"
#include "ecn_baxter/game/events/setup_ended.hpp"
#include "ecn_baxter/game/utils/logger.hpp"
#include <algorithm>
#include <geometry_msgs/TransformStamped.h>
#include <qapplication.h>
#include <tf2/LinearMath/Quaternion.h>

namespace ecn_baxter::game::ros2 {
/// @brief Callback of the acceptance of the action call
/// @param handle the handle to the call to the action
void SetupPointsClient::handle_goal(const PtnSetupHandler::SharedPtr &handle) {
  if (!handle) {
    BAXTER_WARN("Setup rejected bu server");
  }
  BAXTER_INFO("Setup launched !");
}

/// @brief Callback of the last message sent by the action
/// @param result the end result of the action call
void SetupPointsClient::handle_result(
    const PtnSetupHandler::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::ABORTED:
    BAXTER_ERROR("Setup aborted !");
    break;
  case rclcpp_action::ResultCode::CANCELED:
    BAXTER_ERROR("Setup canceled !");
    break;
  case rclcpp_action::ResultCode::UNKNOWN:
    BAXTER_ERROR("Unknown result code :/");
    break;
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  }
  BAXTER_INFO("Setup done with result %d", result.result->success);
  QApplication::sendEvent(events::EventTarget::instance(),
                          new events::SetupEnded());
}

/// @brief Callback of the feedback sent by the setuping action
/// @param handle the handle to the call to the action
/// @param feedback the position and name of the next POI
void SetupPointsClient::handle_feedback(
    [[maybe_unused]] PtnSetupHandler::SharedPtr handle,
    const std::shared_ptr<const PointsSetup::Feedback> feedback) {
  auto p_name = feedback->ptn_name;
  auto p = feedback->ptn;
  BAXTER_INFO("Receiving marker %s at (%f, %f, %f)", p_name.c_str(), p.x, p.y,
              p.z);

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
bool SetupPointsClient::make_action_call() {

  // Verifying that the game props are still existent
  if (game_props.expired()) {
    BAXTER_WARN("Game Properties ptr is expired !");
    return false;
  }

  auto props = game_props.lock();

  BAXTER_INFO("Launching setup phase !");

  // Setup MSG
  auto setup_msg = PointsSetup::Goal();
  setup_msg.ptns_name = std::vector<std::string>(0);
  setup_msg.sides = std::vector<bool>(0);

  BAXTER_INFO("Preparing %zu markers to setup",
              props->setup.needed_points.size());
  for (auto point : props->setup.needed_points) {
    setup_msg.ptns_name.push_back(point.name);
    setup_msg.sides.push_back(game::data::side2bool(point.arm_side));
  }
  return launch_action(setup_msg);
}

} // namespace ecn_baxter::game::ros2

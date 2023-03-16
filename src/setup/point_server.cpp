/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  08/03/2023
 * @description    :  Setup server part (POINT SETUPING)
 *========================================================================**/
#include "ecn_baxter/game/data/arm_side.hpp"
#include <baxter_core_msgs/msg/detail/digital_io_state__struct.hpp>
#include <ecn_baxter/setup/point_server.hpp>
#include <rclcpp/node.hpp>
#include <tf2/time.h>

namespace ecn_baxter::setup {

PointServer::PointServer(sptr<rclcpp::Node> node)
    : BaseServer(node), tf_buffer(node->get_clock()), tf_listener(tf_buffer) {
  button_left_sub = node->create_subscription<DigitalIOState>(
      BUTTON_LEFT_TOPIC, QOS, [&](const DigitalIOState::UniquePtr msg) {
        button_callback(msg, ArmSide::LEFT_ARM);
      });
  button_right_sub = node->create_subscription<DigitalIOState>(
      BUTTON_RIGHT_TOPIC, QOS, [&](const DigitalIOState::UniquePtr msg) {
        button_callback(msg, ArmSide::RIGHT_ARM);
      });
}

/// @brief Get the callback of the buttons states to determine whether we are
/// validating the point or not
/// @param msg the state of the buttons
/// @param the the side from which the state comes
void PointServer::button_callback(const DigitalIOState::UniquePtr &msg,
                                  ArmSide side) {
  switch (side) {
  case ArmSide::LEFT_ARM: {
    if (msg->state == msg->PRESSED) {
      left_button.pressed = true;
    } else {
      left_button.pressed = false;
      left_button.used = false;
    }
  } break;
  case ArmSide::RIGHT_ARM: {
    if (msg->state == msg->PRESSED) {
      right_button.pressed = true;
    } else {
      right_button.pressed = false;
      right_button.used = false;
    }
  } break;
  default:
    break;
  }
}

ra::CancelResponse
PointServer::handle_cancel([[maybe_unused]] const sptr<PointHandle> handle) {
  std::cout << "Request to cancel goal" << std::endl;
  stop = 1;
  return rclcpp_action::CancelResponse::ACCEPT;
}

ra::GoalResponse PointServer::handle_goal(
    [[maybe_unused]] const ra::GoalUUID &uuid,
    [[maybe_unused]] sptr<const action::PointsSetup::Goal> goal) {
  std::cout << "Request to setup" << goal->ptns_name.size() << "points !"
            << std::endl;
  stop = 0;
  return ra::GoalResponse::ACCEPT_AND_EXECUTE;
}

void PointServer::exec(const sptr<PointHandle> handle) {

  using geometry_msgs::msg::Point;

  auto goal = handle->get_goal();
  auto feedback = std::make_shared<action::PointsSetup::Feedback>();

  for (unsigned long i = 0; (i < handle->get_goal()->ptns_name.size()) && !stop;
       i++) {
    ArmSide side = bool2side(goal->sides[i]);

    while (!stop) {
      if ((side == ArmSide::RIGHT_ARM)
              ? (right_button.pressed && !right_button.used)
              : (left_button.pressed && !left_button.used)) {

        // Set buttons as used
        if (side == ArmSide::RIGHT_ARM)
          right_button.used = true;
        else
          left_button.used = true;

        //? LOGGING
        std::cout << "\t✎ Making point #" << i + 1
                  << ((side == ArmSide::RIGHT_ARM) ? "R " : "L ") << "("
                  << goal->ptns_name[i] << ")" << std::endl;

        feedback->ptn_name = goal->ptns_name[i];
        feedback->step = i + 1;

        //? TF Call
        if (tf_buffer.canTransform(
                (side == ArmSide::RIGHT_ARM) ? ROBOT_RIGHT_GRIPPER
                                             : ROBOT_LEFT_GRIPPER,
                ROBOT_BASE, tf2::TimePointZero, tf2::durationFromSec(1.0))) {
          TransformStamped trans = tf_buffer.lookupTransform(
              ROBOT_BASE,
              (side == ArmSide::RIGHT_ARM) ? ROBOT_RIGHT_GRIPPER
                                           : ROBOT_LEFT_GRIPPER,
              tf2::TimePointZero, tf2::durationFromSec(1.0));

          //? Set feeback
          feedback->ptn.x = trans.transform.translation.x;
          feedback->ptn.y = trans.transform.translation.y;
          feedback->ptn.z = trans.transform.translation.z;
        }
        handle->publish_feedback(feedback);
      }
    }
  }

  if (rclcpp::ok()) {
    auto result = std::make_shared<action::PointsSetup::Result>();
    result->success = 0;
    handle->succeed(result);
  }
}

} // namespace ecn_baxter::setup
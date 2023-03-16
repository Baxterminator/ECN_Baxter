/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  08/03/2023
 * @description    :  Setup server part (POINT SETUPING)
 *========================================================================**/
#ifndef POINT_SERVER
#define POINT_SERVER

#include "ecn_baxter/game/data/arm_side.hpp"
#include <baxter_core_msgs/msg/detail/digital_io_state__struct.hpp>
#include <baxter_core_msgs/msg/digital_io_state.hpp>
#include <bits/types/sig_atomic_t.h>
#include <ecn_baxter/action/points_setup.hpp>
#include <ecn_baxter/base/base_server.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ecn_baxter::setup {
using PointHandle = ra::ServerGoalHandle<action::PointsSetup>;
using baxter_core_msgs::msg::DigitalIOState;
using ecn_baxter::game::data::ArmSide;
using ecn_baxter::game::data::bool2side;
using geometry_msgs::msg::TransformStamped;

struct ButtonState {
  bool pressed = false;
  bool used = false;
};

class PointServer : public base::BaseServer<action::PointsSetup> {
private:
  //*==================    TF    =================*/
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  static constexpr auto ROBOT_BASE{"base"};
  static constexpr auto ROBOT_RIGHT_GRIPPER{"right_gripper"};
  static constexpr auto ROBOT_LEFT_GRIPPER{"left_gripper"};

  //*================== BUTTONS =================*/
  static constexpr auto QOS{10};
  static constexpr auto BUTTON_LEFT_TOPIC{
      "/robot/digital_io/left_lower_button/state"};
  static constexpr auto BUTTON_RIGHT_TOPIC{
      "/robot/digital_io/right_lower_button/state"};
  rclcpp::Subscription<DigitalIOState>::SharedPtr button_left_sub;
  rclcpp::Subscription<DigitalIOState>::SharedPtr button_right_sub;
  ButtonState right_button{}, left_button{};

  void button_callback(const DigitalIOState::UniquePtr &, ArmSide);

  //*================== STOP CMD =================*/
  sig_atomic_t stop = 0;

protected:
  /*================== SERVER PART =================*/
  ra::CancelResponse
  handle_cancel([[maybe_unused]] const sptr<PointHandle> handle) override;
  ra::GoalResponse handle_goal(
      [[maybe_unused]] const ra::GoalUUID &uuid,
      [[maybe_unused]] sptr<const action::PointsSetup::Goal> goal) override;
  void exec(const sptr<PointHandle> handle) override;

public:
  PointServer(sptr<rclcpp::Node>);
};
} // namespace ecn_baxter::setup

#endif
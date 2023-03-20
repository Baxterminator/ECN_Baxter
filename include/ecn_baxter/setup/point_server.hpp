/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  08/03/2023
 * @description    :  Setup server part (POINT SETUPING)
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_POINT_SERVER
#define ECN_BAXTER_POINT_SERVER

#include "ecn_baxter/action/points_setup.hpp"
#include "ecn_baxter/base/base_server.hpp"
#include "ecn_baxter/game/data/arm_side.hpp"
#include <baxter_core_msgs/msg/digital_io_state.hpp>
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
using geometry_msgs::msg::TransformStamped;

struct ButtonState {
  bool pressed = false;
  bool used = false;
};

class PointServer : public ecn::base::BaseServer<action::PointsSetup> {
private:
  /**═════════════════════════════════════════════════════════════════════════
   *?                                 TF
   * ═════════════════════════════════════════════════════════════════════════*/

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  static constexpr auto ROBOT_BASE{"base"};
  static constexpr auto ROBOT_RIGHT_GRIPPER{"right_gripper"};
  static constexpr auto ROBOT_LEFT_GRIPPER{"left_gripper"};

  /**═════════════════════════════════════════════════════════════════════════
   *?                                Buttons
   * ═════════════════════════════════════════════════════════════════════════*/

  static constexpr auto QOS{10};
  static constexpr auto BUTTON_LEFT_TOPIC{
      "/robot/digital_io/left_lower_button/state"};
  static constexpr auto BUTTON_RIGHT_TOPIC{
      "/robot/digital_io/right_lower_button/state"};
  rclcpp::Subscription<DigitalIOState>::SharedPtr left_sub, right_sub;
  ButtonState right_button{}, left_button{};

  void button_callback(const DigitalIOState::UniquePtr &, ArmSide);

protected:
  /**═════════════════════════════════════════════════════════════════════════
   *!                                Server Part
   * ═════════════════════════════════════════════════════════════════════════*/

  ra::CancelResponse handle_cancel(
      [[maybe_unused]] const std::shared_ptr<PointHandle> handle) override;
  ra::GoalResponse
  handle_goal([[maybe_unused]] const ra::GoalUUID &uuid,
              [[maybe_unused]] std::shared_ptr<const action::PointsSetup::Goal>
                  goal) override;
  void exec(const std::shared_ptr<PointHandle> handle) override;

public:
  static constexpr auto SERVER_NAME{"points_setup"};
  PointServer(std::shared_ptr<rclcpp::Node>);
};
} // namespace ecn_baxter::setup

#endif
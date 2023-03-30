/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  28/03/2023
 * @description    :  Script to set the baxter in at specific joints position
 * @version        :  rev 23w12.4
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/utils/joint_states_wrapper.hpp"
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

using namespace std::chrono_literals;
using baxter_core_msgs::msg::JointCommand;
using sensor_msgs::msg::JointState;

namespace ecn_baxter::utils {

class ArmsPosition : public rclcpp::Node {
public:
  ArmsPosition(rclcpp::NodeOptions opts) : rclcpp::Node("arms_position", opts) {
    set_left_pos();
    set_right_pos();

    threshold = declare_parameter("threshold", 1E-2);

    pub_l = create_publisher<JointCommand>(LEFT_ARM_TOPIC, 10);
    pub_r = create_publisher<JointCommand>(RIGHT_ARM_TOPIC, 10);

    sub_jstates = create_subscription<JointState>(
        JOINT_STATE_TOPIC, 10,
        [&](const JointState::SharedPtr msg) { j_state.set(msg); });
  }

  double error_left() { return is_close(j_state, false); }
  double error_right() { return is_close(j_state, true); }
  void publish_left() { pub_l->publish(pos_left); }
  void publish_right() { pub_r->publish(pos_right); }

  double get_threshold() { return threshold; }

private:
  ecn_baxter::utils::BaxterJoints j_state;
  JointCommand pos_right, pos_left;
  double threshold;
  rclcpp::Publisher<JointCommand>::SharedPtr pub_l, pub_r;
  rclcpp::Subscription<JointState>::SharedPtr sub_jstates;
  static constexpr auto LEFT_ARM_TOPIC{"/robot/limb/left/joint_command"};
  static constexpr auto RIGHT_ARM_TOPIC{"/robot/limb/right/joint_command"};
  static constexpr auto JOINT_STATE_TOPIC{"/robot/joint_states"};

  ///@brief Set the left arm position from parameters
  void set_left_pos() {
    pos_left.mode = JointCommand::POSITION_MODE;
    make_names("left", pos_left.names);
    pos_left.command.resize(7);
    pos_left.command[0] = declare_parameter("left_s0", -0.08);
    pos_left.command[1] = declare_parameter("left_s1", -1.0);
    pos_left.command[2] = declare_parameter("left_e0", -1.19);
    pos_left.command[3] = declare_parameter("left_e1", -1.94);
    pos_left.command[4] = declare_parameter("left_w0", -0.67);
    pos_left.command[5] = declare_parameter("left_w1", -1.03);
    pos_left.command[6] = declare_parameter("left_w2", -0.50);
  }

  ///@brief Set the right arm position from parameters
  void set_right_pos() {
    pos_right.mode = JointCommand::POSITION_MODE;
    make_names("right", pos_left.names);
    pos_right.command.resize(7);
    pos_right.command[0] = declare_parameter("right_s0", 0.08);
    pos_right.command[1] = declare_parameter("right_s1", -1.0);
    pos_right.command[2] = declare_parameter("right_e0", 1.19);
    pos_right.command[3] = declare_parameter("right_e1", 1.94);
    pos_right.command[4] = declare_parameter("right_w0", -0.67);
    pos_right.command[5] = declare_parameter("right_w1", 1.03);
    pos_right.command[6] = declare_parameter("right_w2", 0.50);
  }

  double is_close(const ecn_baxter::utils::BaxterJoints &act, bool right) {
    if (right) {
      return (pow(act.right.s0.pos - pos_right.command[0], 2) +
              pow(act.right.s1.pos - pos_right.command[1], 2) +
              pow(act.right.e0.pos - pos_right.command[2], 2) +
              pow(act.right.e1.pos - pos_right.command[3], 2) +
              pow(act.right.w0.pos - pos_right.command[4], 2) +
              pow(act.right.w1.pos - pos_right.command[5], 2) +
              pow(act.right.w2.pos - pos_right.command[6], 2));
    }
    return (pow(act.left.e0.pos - pos_left.command[0], 2) +
            pow(act.left.s1.pos - pos_left.command[1], 2) +
            pow(act.left.e0.pos - pos_left.command[2], 2) +
            pow(act.left.e1.pos - pos_left.command[3], 2) +
            pow(act.left.w0.pos - pos_left.command[4], 2) +
            pow(act.left.w1.pos - pos_left.command[5], 2) +
            pow(act.left.w2.pos - pos_left.command[6], 2));
  }
};
} // namespace ecn_baxter::utils

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node =
      std::make_shared<ecn_baxter::utils::ArmsPosition>(rclcpp::NodeOptions{});

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);

  bool close = false;
  do {
    ex.spin_once(10ms);

    close = true;
    if (node->error_left() > 1E-2) {
      node->publish_left();
      close = false;
    }
    if (node->error_right() > 1E-2) {
      node->publish_right();
      close = false;
    }
  } while (rclcpp::ok() && !close);

  rclcpp::shutdown();
}
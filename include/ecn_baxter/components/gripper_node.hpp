#ifndef ECN_BAXTER_GRIPPER_NODE_H
#define ECN_BAXTER_GRIPPER_NODE_H

#include "baxter_core_msgs/msg/end_effector_command.hpp"
#include "baxter_core_msgs/msg/end_effector_state.hpp"
#include "ecn_baxter/msg/baxter_action.hpp"
#include "ecn_baxter/msg/detail/vacuum_sensor__struct.hpp"
#include "ecn_baxter/msg/vacuum_sensor.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ecn_baxter/async_timer.hpp"
#include "rapidjson/document.h"

#include <chrono>
#include <thread>

namespace ecn_baxter {
using baxter_core_msgs::msg::EndEffectorCommand;
using baxter_core_msgs::msg::EndEffectorState;
using ecn_baxter::msg::BaxterAction;
using ecn_baxter::msg::VacuumSensor;

using namespace std::this_thread;
using namespace std::chrono_literals;
using namespace std::chrono;
using namespace rapidjson;

enum GripperType { CLAMP, VACUUM };

/**
 * Tool for using Baxter's Grippers
 */
class GripperNode : public rclcpp::Node {
public:
  explicit GripperNode(const rclcpp::NodeOptions &options);

protected:
  // Gripper properties
  bool _initialized = false;
  std::string _side;
  GripperType _type = CLAMP;
  unsigned char _calibrated = false;

  // Gripper latest_state
  std::string latest_state_src;
  std::string latest_cmd;
  Document latest_state;

  // Communication
  static constexpr int QOS = 10;
  static constexpr auto ACTION_TOPIC = "/robot/gripper";
  std::string CMD_TOPIC;
  std::string STATE_TOPIC;
  std::string VACUUM_TOPIC;

  // Vacuum checker properties
  milliseconds timer_duration = 500ms;
  milliseconds check_delay = 200ms;
  milliseconds timer_loop_time = 500ms;
  static constexpr auto VACUUM_SENSOR = "vacuum sensor";
  static constexpr auto VACUUM_THRESHOLD = "vacuum threshold";
  AsyncTimer checker;

  // Methods
  void topics_init();
  void action_process(const BaxterAction::UniquePtr &msg);
  void state_process(const EndEffectorState::UniquePtr &msg);
  void initialize(const EndEffectorState::UniquePtr &msg);

  void vacuum_check();

  void parse_state();

  void action(const std::string &action_id);
  void reset() { action("reset"); };
  void calibrate() { action("calibrate"); };
  void release() { action("release"); };

  // Topic-related members
  rclcpp::Subscription<BaxterAction>::SharedPtr act_sub;
  rclcpp::Subscription<EndEffectorState>::SharedPtr state_sub;
  rclcpp::Publisher<EndEffectorCommand>::SharedPtr cmd_pub;
  rclcpp::Publisher<VacuumSensor>::SharedPtr vacuum_pub;
  rclcpp::TimerBase::SharedPtr vacuum_checker;

  EndEffectorCommand cmd;
};
} // namespace ecn_baxter
#endif // ECN_BAXTER_GRIPPER_NODE_H

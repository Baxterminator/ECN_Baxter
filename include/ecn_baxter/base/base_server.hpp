/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  08/03/2023
 * @description    :  Base action server
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BASE_SERVER
#define ECN_BASE_SERVER

#include <bits/types/sig_atomic_t.h>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <thread>

namespace ra = rclcpp_action;

namespace ecn::base {

template <typename ActionT> using ActionHandle = ra::ServerGoalHandle<ActionT>;

/// @brief Base for action server
/// @tparam ActionT the action type
template <typename ActionT> class BaseServer {
private:
  std::shared_ptr<ra::Server<ActionT>> server;
  sig_atomic_t stop_cmd = 0;

  /// @brief Accept the action, start the thread
  void handle_accepted(const std::shared_ptr<ActionHandle<ActionT>> handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&BaseServer<ActionT>::exec, this, _1), handle}
        .detach();
  }

protected:
  /// @brief Process a cancelling event
  virtual ra::CancelResponse
  handle_cancel(const std::shared_ptr<ActionHandle<ActionT>> handle) = 0;

  /// @brief Process an action request
  virtual ra::GoalResponse handle_goal(
      [[maybe_unused]] const ra::GoalUUID &uuid,
      [[maybe_unused]] std::shared_ptr<const typename ActionT::Goal> goal) = 0;

  /// @brief Core action work
  virtual void exec(const std::shared_ptr<ActionHandle<ActionT>> handle) = 0;
  inline bool has_to_stop() { return stop_cmd == 1; }

public:
  BaseServer(std::shared_ptr<rclcpp::Node> node, const char *const name) {
    using namespace std::placeholders;

    server = ra::create_server<ActionT>(
        node, name, std::bind(&BaseServer<ActionT>::handle_goal, this, _1, _2),
        std::bind(&BaseServer<ActionT>::handle_cancel, this, _1),
        std::bind(&BaseServer<ActionT>::handle_accepted, this, _1));
  }
  virtual ~BaseServer() { server.reset(); };
  inline void stop_server() { stop_cmd = 1; };
};
} // namespace ecn::base
#endif
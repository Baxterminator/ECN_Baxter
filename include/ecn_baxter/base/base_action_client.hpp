/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  21/03/2023
 * @description    :  Base action client
 * @version        :  rev 23w12.3
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BASE_ACTION_CLIENT
#define ECN_BASE_ACTION_CLIENT

#include <chrono>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ra = rclcpp_action;
using namespace std::chrono_literals;

namespace ecn::base {

template <typename ActionT> using ActionHandle = ra::ClientGoalHandle<ActionT>;
template <typename ActionT>
using ActionWrappedResult = typename ActionHandle<ActionT>::WrappedResult;
template <typename ActionT> using ActionFeedback = typename ActionT::Feedback;
template <typename ActionT> using ActionGoal = typename ActionT::Goal;
template <typename ActionT>
using ActionGoalOpts = typename ra::Client<ActionT>::SendGoalOptions;

/// @brief Abstraction class for action client
/// @tparam ActionT the action type
template <typename ActionT> class BaseActionClient {
protected:
  std::weak_ptr<rclcpp::Node> node_handle;
  std::shared_ptr<ra::Client<ActionT>> client;

#ifdef LEGACY_IDL
  /// @brief Callback of the acceptance of the action call
  /// @param future the future linked to the call of the action
  /// @deprecated This function is only for legacy ROS2 version support (foxy,
  /// galactic, ...). Use PtnSetupHandler::SharedPtr from humble and onward
  inline void handle_goal(
      std::shared_future<std::shared_ptr<ActionHandle<ActionT>>> future) {
    auto handle = future.get();
    handle_goal(handle);
  }
#endif
  virtual void handle_goal(const std::shared_ptr<ActionHandle<ActionT>> &) = 0;
  virtual void
  handle_feedback(std::shared_ptr<ActionHandle<ActionT>>,
                  const std::shared_ptr<const ActionFeedback<ActionT>>) = 0;
  virtual void handle_result(const ActionWrappedResult<ActionT> &) = 0;

  bool launch_action(ActionGoal<ActionT> &goal) {

    // If the server is not online, stop here
    if (!client->wait_for_action_server(150ms)) {
      if (!node_handle.expired()) {
        RCLCPP_ERROR(node_handle.lock()->get_logger(),
                     "Action server not available after waiting");
      }
      return false;
    }

    auto goal_opts = ActionGoalOpts<ActionT>();
#ifdef LEGACY_IDL
    goal_opts.goal_response_callback =
        [&](const std::shared_future<std::shared_ptr<ActionHandle<ActionT>>>
                future) { handle_goal(future); };
#else
    goal_opts.goal_response_callback =
        [&](const std::shared_ptr<ActionHandle<ActionT>> &handle) {
          handle_goal(handle);
        };
#endif
    goal_opts.feedback_callback =
        [&](std::shared_ptr<ActionHandle<ActionT>> handle,
            const std::shared_ptr<const ActionFeedback<ActionT>> feedback) {
          handle_feedback(handle, feedback);
        };
    goal_opts.result_callback =
        [&](const ActionWrappedResult<ActionT> &result) {
          handle_result(result);
        };

    auto h = client->async_send_goal(goal, goal_opts);
    return true;
  }

public:
  BaseActionClient(std::shared_ptr<rclcpp::Node> node,
                   [[maybe_unused]] const std::string &service_name)
      : node_handle(node) {
    client = ra::create_client<ActionT>(node, service_name);
  }
  virtual bool make_action_call() = 0;

  virtual ~BaseActionClient() { client->async_cancel_all_goals(); }
};

} // namespace ecn::base

#endif
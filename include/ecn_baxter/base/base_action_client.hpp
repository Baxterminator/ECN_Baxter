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

#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ra = rclcpp_action;

namespace ecn::base {

template <typename ActionT> using ActionHandle = ra::ClientGoalHandle<ActionT>;
template <typename ActionT>
using ActionWrappedResult = typename ActionHandle<ActionT>::WrappedResult;
template <typename ActionT> using ActionFeedback = typename ActionT::Feedback;
template <typename ActionT> using ActionGoal = typename ActionT::Goal;
template <typename ActionT>
using ActionGoalOpts = typename ra::Client<ActionT>::SendGoalOptions;

/// @brief Base for action client
/// @tparam ActionT the action type
template <typename ActionT> class BaseActionClient {
private:
  std::weak_ptr<rclcpp::Node> node_handle;
  std::shared_ptr<ra::Client<ActionT>> client;

protected:
#ifdef LEGACY_IDL
  /// @brief Callback of the acceptance of the action call
  /// @param future the future linked to the call of the action
  /// @deprecated This function is only for legacy ROS2 version support (foxy,
  /// galactic, ...). Use PtnSetupHandler::SharedPtr from humble and onward
  inline void
  handle_goal(std::shared_future<PtnSetupHandler::SharedPtr> future) {
    auto handle = future.get();
    handle_goal(handle);
  }
#endif
  virtual void handle_goal(const std::shared_ptr<ActionHandle<ActionT>>) = 0;
  virtual void
  handle_feedback(std::shared_ptr<ActionHandle<ActionT>>,
                  const std::shared_ptr<const ActionFeedback<ActionT>>) = 0;
  virtual void handle_result(const ActionWrappedResult<ActionT>) = 0;

  void launch_action(ActionGoal<ActionT> &goal) {

    auto goal_opts = ActionGoalOpts<ActionT>();

    goal_opts.goal_response_callback =
        [&](const std::shared_ptr<ActionHandle<ActionT>> &handle) {
          handle_goal(handle);
        };
    goal_opts.feedback_callback =
        [&](std::shared_ptr<ActionHandle<ActionT>> handle,
            const std::shared_ptr<const ActionFeedback<ActionT>> feedback) {
          handle_feedback(handle, feedback);
        };
    goal_opts.result_callback =
        [&](const ActionWrappedResult<ActionT> &result) {
          handle_result(result);
        };

    client->async_send_goal(goal, goal_opts);
  }

public:
  BaseActionClient(std::shared_ptr<rclcpp::Node> node,
                   const std::string &service_name)
      : node_handle(node) {
    client = ra::create_client<ActionT>(node, service_name);
  }
  virtual void make_action_call() = 0;
};

} // namespace ecn::base

#endif
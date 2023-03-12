/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  08/03/2023
 * @description    :  Base server for setuping
 *========================================================================**/
#ifndef BASE_SERVER
#define BASE_SERVER

#include <bits/types/sig_atomic_t.h>
#include <ecn_baxter/utils.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace ra = rclcpp_action;

namespace ecn_baxter::setup {

template <typename ActionT> using ActionHandle = ra::ServerGoalHandle<ActionT>;

template <typename ActionT> class BaseServer {
private:
  sptr<ra::Server<ActionT>> server;
  sig_atomic_t stop_cmd = 0;
  void handle_accepted(const sptr<ActionHandle<ActionT>> handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&BaseServer<ActionT>::exec, this, _1), handle}
        .detach();
  }

protected:
  virtual ra::CancelResponse
  handle_cancel(const sptr<ActionHandle<ActionT>> handle) = 0;

  virtual ra::GoalResponse
  handle_goal([[maybe_unused]] const ra::GoalUUID &uuid,
              [[maybe_unused]] sptr<const typename ActionT::Goal> goal) = 0;

  virtual void exec(const sptr<ActionHandle<ActionT>> handle) = 0;
  inline bool has_to_stop() { return stop_cmd == 1; }

public:
  BaseServer(sptr<rclcpp::Node> node) {
    using namespace std::placeholders;

    server = ra::create_server<ActionT>(
        node, "server",
        std::bind(&BaseServer<ActionT>::handle_goal, this, _1, _2),
        std::bind(&BaseServer<ActionT>::handle_cancel, this, _1),
        std::bind(&BaseServer<ActionT>::handle_accepted, this, _1));
  }
  virtual ~BaseServer() { server.reset(); };
  inline void stop_server() { stop_cmd = 1; };
};
} // namespace ecn_baxter::setup
#endif
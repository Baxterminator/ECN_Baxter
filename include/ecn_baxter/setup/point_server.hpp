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

#include <ecn_baxter/action/points_setup.hpp>
#include <ecn_baxter/setup/base_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>

namespace ecn_baxter::setup {
using PointHandle = ra::ServerGoalHandle<action::PointsSetup>;

class PointServer : public BaseServer<action::PointsSetup> {
protected:
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
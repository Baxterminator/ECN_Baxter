/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  08/03/2023
 * @description    :  Setup server part (POINT SETUPING)
 *========================================================================**/
#include <ecn_baxter/setup/point_server.hpp>
#include <rclcpp/node.hpp>

namespace ecn_baxter::setup {

PointServer::PointServer(sptr<rclcpp::Node> node) : BaseServer(node) {}

ra::CancelResponse
PointServer::handle_cancel([[maybe_unused]] const sptr<PointHandle> handle) {
  std::cout << "Request to cancel goal" << std::endl;
  return rclcpp_action::CancelResponse::ACCEPT;
}

ra::GoalResponse PointServer::handle_goal(
    [[maybe_unused]] const ra::GoalUUID &uuid,
    [[maybe_unused]] sptr<const action::PointsSetup::Goal> goal) {
  std::cout << "Request to setup" << goal->ptns_name.size() << "points !"
            << std::endl;
  return ra::GoalResponse::ACCEPT_AND_EXECUTE;
}

void PointServer::exec(const sptr<PointHandle> handle) {

  using geometry_msgs::msg::Point;

  auto goal = handle->get_goal();
  auto feedback = std::make_shared<action::PointsSetup::Feedback>();

  for (unsigned long i = 0; i < handle->get_goal()->ptns_name.size(); i++) {
    std::cout << "Making point #" << i + 1 << "(" << goal->ptns_name[i] << ")"
              << std::endl;
    feedback->ptn_name = goal->ptns_name[i];
    feedback->step = i + 1;
    Point p;
    p.x = 50;
    p.y = 10;
    p.z = 5;
    feedback->ptn = p;

    handle->publish_feedback(feedback);
  }

  if (rclcpp::ok()) {
    auto result = std::make_shared<action::PointsSetup::Result>();
    result->success = 0;
    handle->succeed(result);
  }
}

} // namespace ecn_baxter::setup
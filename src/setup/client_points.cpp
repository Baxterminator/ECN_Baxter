/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS 2 Node part for point setuping
 *========================================================================**/
#include <ecn_baxter/setup/client_points.hpp>

namespace ecn_baxter::setup {
/// @brief Callback of the acceptance of the action call
/// @param handle the handle to the call to the action
void SetupPointsClient::ptn_setup_goal(
    const PtnSetupHandler::SharedPtr &handle) {
  if (!handle) {
    RCLCPP_ERROR(logger, "Setup rejected bu server");
  }
  RCLCPP_INFO(logger, "Setup launched !");
}

/// @brief Callback of the last message sent by the action
/// @param result the end result of the action call
void SetupPointsClient::ptn_setup_result(
    const PtnSetupHandler::WrappedResult &result) {
  switch (result.code) {
  case ResultCode::ABORTED:
    RCLCPP_ERROR(logger, "Setup aborted !");
    break;
  case ResultCode::CANCELED:
    RCLCPP_ERROR(logger, "Setup canceled !");
    break;
  case ResultCode::UNKNOWN:
    RCLCPP_ERROR(logger, "Unknown result code :/");
    break;
  }
  RCLCPP_INFO(logger, "Setup done with result %d", result.result->success);
}

/// @brief Callback of the feedback sent by the setuping action
/// @param handle the handle to the call to the action
/// @param feedback the position and name of the next POI
void SetupPointsClient::ptn_setup_feedback(
    PtnSetupHandler::SharedPtr &handle,
    const sptr<const PointsSetup::Feedback> feedback) {
  auto p_name = feedback->ptn_name;
  auto p = feedback->ptn;
  RCLCPP_INFO(logger, "Receiving marker %s at (%f, %f, %f)", p_name.c_str(),
              p.x, p.y, p.z);
}

} // namespace ecn_baxter::setup

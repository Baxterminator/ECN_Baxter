/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS2 part of the GameMaster
 *========================================================================**/

#ifndef GAME_MASTER_2_HPP
#define GAME_MASTER_2_HPP

#include <ecn_baxter/action/points_setup.hpp>
#include <ecn_baxter/setup/client_points.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ecn_baxter::game {

using ecn_baxter::action::PointsSetup;

class GameMaster_2 : public rclcpp::Node,
                     public ecn_baxter::setup::SetupPointsClient {
public:
  explicit GameMaster_2(rclcpp::NodeOptions opts);
};
} // namespace ecn_baxter::game
#endif // GAME_MASTER_2_HPP
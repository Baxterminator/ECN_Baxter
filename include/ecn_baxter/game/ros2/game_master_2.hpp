/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS2 part of the GameMaster
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_GAME_MASTER_2_HPP
#define ECN_BAXTER_GAME_MASTER_2_HPP

#include "ecn_baxter/action/points_setup.hpp"
#include "ecn_baxter/game/ros2/client_points.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ecn_baxter::game::ros2 {

using ecn_baxter::action::PointsSetup;

class GameMaster_2 : public rclcpp::Node, public SetupPointsClient {
public:
  explicit GameMaster_2(rclcpp::NodeOptions opts);
};
} // namespace ecn_baxter::game::ros2
#endif
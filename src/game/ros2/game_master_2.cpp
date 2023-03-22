/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS2 part of the GameMaster
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/ros2/game_master_2.hpp"
#include "ecn_baxter/action/detail/points_setup__struct.hpp"
#include "ecn_baxter/game/ros2/client_points.hpp"

namespace ecn_baxter::game::ros2 {

GameMaster_2::GameMaster_2(rclcpp::NodeOptions opts)
    : rclcpp::Node("game_master_2", opts) {
  points_setup = std::make_shared<SetupPointsClient>(shared_from_this());
}

} // namespace ecn_baxter::game::ros2
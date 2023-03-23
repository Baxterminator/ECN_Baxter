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
#include "ecn_baxter/game/data/game_properties.hpp"
#include "ecn_baxter/game/ros2/client_points.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace ecn_baxter::game::ros2 {

using ecn_baxter::action::PointsSetup;

class GameMaster_2 : public rclcpp::Node {
public:
  explicit GameMaster_2(rclcpp::NodeOptions opts);

  void update_game_props(std::shared_ptr<data::GameProperties> gprops) {
    points_setup->update_ptn_setup_client(gprops);
  }

  void set_handle(std::shared_ptr<rclcpp::Node>);

  bool make_setup() { return points_setup->make_action_call(); }

protected:
  std::shared_ptr<SetupPointsClient> points_setup = nullptr;
};
} // namespace ecn_baxter::game::ros2
#endif
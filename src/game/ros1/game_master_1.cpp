/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS1 part of the GameMaster
 * @version        :  rev 23w12.2
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/ros1/game_master_1.hpp"
#include "ecn_baxter/game/data/game_players.hpp"
#include "ros/node_handle.h"

namespace ecn_baxter::game::ros1 {

GameMaster_1::GameMaster_1(std::shared_ptr<data::PlayerList> list)
    : BridgesManager(), TFBroadcaster() {
  _handle = std::make_shared<ros::NodeHandle>();
  tf_broadcast_init(_handle);
  bridges_init(_handle, list);
}

} // namespace ecn_baxter::game::ros1
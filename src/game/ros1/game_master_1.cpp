/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS1 part of the GameMaster
 *========================================================================**/
#include "ros/node_handle.h"
#include <ecn_baxter/game/ros1/game_master_1.hpp>
#include <memory>

namespace ecn_baxter::game::ros1 {

const std::string GameMaster_1::NODE_NAME = "game_master_1";

/// @brief ROS1 Part of the game master
GameMaster_1::GameMaster_1() : BridgeManager(), TFBroadcaster() {
  _handle = std::make_shared<ros::NodeHandle>();
  tf_broadcast_init(_handle);
  bridges_init(_handle);
}

} // namespace ecn_baxter::game::ros1
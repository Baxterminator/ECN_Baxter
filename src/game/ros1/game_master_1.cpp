/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS1 part of the GameMaster
 *========================================================================**/
#include <ecn_baxter/game/ros1/game_master_1.hpp>

namespace ecn_baxter::game::ros1 {

const std::string GameMaster_1::NODE_NAME = "game_master_1";

/// @brief ROS1 Part of the game master
GameMaster_1::GameMaster_1()
    : _handle(std::make_shared<ros::NodeHandle>()), BridgeLookup(_handle),
      TFBroadcaster(_handle) {}

} // namespace ecn_baxter::game::ros1
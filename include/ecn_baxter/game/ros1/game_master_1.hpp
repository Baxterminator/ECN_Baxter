/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS1 part of the GameMaster
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_GAME_MASTER_1_HPP
#define ECN_BAXTER_GAME_MASTER_1_HPP

#include "ecn_baxter/game/data/game_properties.hpp"
#include "ecn_baxter/game/ros1/bridge_lookup.hpp"
#include "ecn_baxter/game/ros1/tf_broadcast.hpp"
#include <memory>
#include <ros/node_handle.h>

namespace ecn_baxter::game::ros1 {

/// @brief ROS1 Part of the game master
class GameMaster_1 : public BridgesManager, public TFBroadcaster {
public:
  GameMaster_1();
  static constexpr auto NODE_NAME{"game_master_1"};

private:
  std::shared_ptr<ros::NodeHandle> _handle;
};
}; // namespace ecn_baxter::game::ros1

#endif
/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Game Player definition
 *========================================================================**/

#ifndef GAME_PLAYER
#define GAME_PLAYER

#include <ecn_baxter/game/data/arm_side.hpp>
#include <vector>

namespace ecn_baxter::game::data {

struct GamePlayer {
  std::string name, bridge_name;
  bool connected = false;
  bool new_discovered = true;
  ArmSide side = ArmSide::NONE;
  unsigned int row_id = 0;
};

typedef std::vector<GamePlayer> PlayerList;
} // namespace ecn_baxter::game::data

#endif
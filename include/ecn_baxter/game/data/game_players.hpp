/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Game Player definition
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_GAME_PLAYER
#define ECN_BAXTER_GAME_PLAYER

#include "ecn_baxter/game/data/arm_side.hpp"
#include <vector>

namespace ecn_baxter::game::data {

/// @brief Structure describing a game player
struct GamePlayer {
  std::string name, bridge_name;
  bool connected = false;
  bool new_discovered = true;
  ArmSide side = ArmSide::NONE;
  unsigned int row_id = 0;
};

typedef std::vector<GamePlayer> PlayerList;

/// @brief Random player name for blocking the bridge
constexpr auto block_player{"fsjgfdsojlgnwdkjhgsioegj"};

/// @brief Void player name for unblocking the bridge
constexpr auto free_player{""};

} // namespace ecn_baxter::game::data

#endif
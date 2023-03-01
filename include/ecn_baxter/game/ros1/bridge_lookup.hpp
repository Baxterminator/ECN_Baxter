/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Sub-part of the ROS1 game master for checking connected
 *                    players
 *========================================================================**/

#ifndef BRIDGE_LOOKUP
#define BRIDGE_LOOKUP

#include <ecn_baxter/game/structures/game_players.hpp>
#include <ecn_baxter/utils.hpp>
#include <ros/duration.h>
#include <ros/master.h>
#include <ros/node_handle.h>
#include <vector>

#include <sstream>
#include <string>
#include <vector>

namespace ecn_baxter::game::ros1 {

using namespace ecn_baxter::game::structure;

class BridgeLookup {
private:
  const char *bridge_name = "baxter_ros1_bridge_";
  bool is_bridge(const std::string &, std::string &) const;
  void update_connected_players(const std::vector<std::string> &);
  void reset_players_state();
  std::function<void(const PlayerList &)> _callback;

  const ros::WallDuration check_dur{0.1};

protected:
  explicit BridgeLookup(){};

  PlayerList players;
  void look_for_briges();

  //* Timer
  ros::WallTimer _check_timer;

public:
  void bridges_init(sptr<ros::NodeHandle>);
  inline void
  set_look_up_callback(std::function<void(const PlayerList &)> &callback) {
    _callback = callback;
  }
};
} // namespace ecn_baxter::game::ros1

#endif
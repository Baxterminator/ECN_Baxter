/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Sub-part of the ROS1 game master for checking connected
 *                    players
 *========================================================================**/
#include <ecn_baxter/game/ros1/bridge_lookup.hpp>

namespace ecn_baxter::game::ros1 {

void BridgeLookup::bridges_init(sptr<ros::NodeHandle> handle) {
  _check_timer =
      handle->createWallTimer(check_dur, [this](const ros::WallTimerEvent &e) {
        look_for_briges();
        if (_callback != nullptr)
          _callback(players);
      });
}

/// @brief Check if the node name start as a bridge should and set the user name
/// parameter accordingly
bool BridgeLookup::is_bridge(const std::string &node_name,
                             std::string &user) const {
  user = "";
  // If the name is too short to be a bridge, directly return false
  if (strlen(node_name.c_str()) < strlen(bridge_name))
    return false;

  // Else
  std::stringstream ss;
  for (int i = 0; i < strlen(node_name.c_str()); i++) {
    //* Check if is a bridge
    if (i < strlen(bridge_name) && node_name[i] != bridge_name[i])
      return false;
    //* If is a bridge, save the name
    if (i >= strlen(bridge_name))
      ss << node_name[i];
  }

  // Return the computed user
  user = ss.str();
  return true;
}

/// @brief Look out for bridges on the network and save them for later purpose
void BridgeLookup::look_for_briges() {
  std::vector<std::string> current_nodes, current_users;
  ros::master::getNodes(current_nodes);

  auto it = current_nodes.begin();
  std::string user;
  while (it != current_nodes.end()) {
    auto n = it->data();
    if (is_bridge(n, user))
      current_users.push_back(user);
    it++;
  }

  reset_players_state();
  update_connected_players(current_users);
}

/// @brief Reset all players "connected" state to false
void BridgeLookup::reset_players_state() {
  for (auto &p : players)
    p.connected = false;
}

/// @brief Update local players list to last look out
void BridgeLookup::update_connected_players(
    const std::vector<std::string> &to_add) {
  bool found;
  for (auto &p_name : to_add) {

    found = false;

    // If players already in list, update to "connected"
    for (auto &p_saved : players) {
      if (p_saved.name == p_name) {
        p_saved.connected = true;
        found = true;
        break;
      }
    }

    // If not in list, add it
    if (!found)
      players.push_back(GamePlayer{p_name, bridge_name + p_name, true});
  }
}
} // namespace ecn_baxter::game::ros1
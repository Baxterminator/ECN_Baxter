/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Sub-part of the ROS1 game master for checking connected
 *                    players
 * @version        :  rev 23w12.2
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/ros1/bridge_lookup.hpp"
#include "ecn_baxter/game/data/arm_side.hpp"
#include "ecn_baxter/game/data/game_players.hpp"
#include "ecn_baxter/game/events/bridges_update_events.hpp"
#include "ecn_baxter/game/events/event_target.hpp"
#include <memory>
#include <qapplication.h>

namespace ecn_baxter::game::ros1 {

/**════════════════════════════════════════════════════════════════════════
 *!                           Initialization
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Initialize the periodic bridge lookup related objects
void BridgesManager::bridges_init(std::shared_ptr<ros::NodeHandle> handle,
                                  std::shared_ptr<data::PlayerList> players) {
  _players = players;
  _check_timer =
      handle->createWallTimer(check_dur, [this](const ros::WallTimerEvent &) {
        look_for_briges();
        events::BridgesUpdate ev;
        QApplication::sendEvent(events::EventTarget::instance(), &ev);
      });

  _slave_client =
      handle->serviceClient<baxter_core_msgs::BridgePublishersForce>(
          slave_service);
}

/**════════════════════════════════════════════════════════════════════════
 **                                Utils
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Extract the user name from a bridge name, if it doesn't match with a
/// bridge, it return ""
std::string BridgesManager::extract_name(const std::string &full_name) const {
  int i = full_name.length() - 1;
  for (; i >= 0; i--) {
    if ((char)full_name[i] == '/')
      break;
  }
  return full_name.substr(i + 1);
}

/// @brief Check if the node name start as a bridge should and set the user name
/// parameter accordingly
bool BridgesManager::is_bridge(const std::string &node_name,
                               std::string &user) const {
  user = "";
  // If the name is too short to be a bridge, directly return false
  if (strlen(node_name.c_str()) < strlen(bridge_name))
    return false;

  // Else
  std::stringstream ss;
  for (unsigned long i = 0; i < strlen(node_name.c_str()); i++) {
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

/**════════════════════════════════════════════════════════════════════════
 *?                            Bridge Lookup
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Look out for bridges on the network and save them for later purpose
void BridgesManager::look_for_briges() {
  std::vector<std::string> current_nodes, current_users;
  ros::master::getNodes(current_nodes);

  auto it = current_nodes.begin();
  std::string user;
  while (it != current_nodes.end()) {
    auto n = extract_name(it->data());
    if (is_bridge(n, user)) {
      current_users.push_back(user);
    }
    it++;
  }

  reset_players_state();
  update_connected_players(current_users);
}

/// @brief Reset all players "connected" state to false
void BridgesManager::reset_players_state() {
  if (!_players.expired()) {
    auto players = _players.lock();
    auto i = players->begin();
    while (i != players->end()) {
      i->connected = false;
      i++;
    }
  }
}

/// @brief Update local players list to last look out
void BridgesManager::update_connected_players(
    const std::vector<std::string> &to_add) {
  if (_players.expired())
    return;

  auto players = _players.lock();

  bool found;
  for (auto &p_name : to_add) {

    found = false;

    // If players already in list, update to "connected"
    auto p_saved = players->begin();
    while (p_saved != players->end()) {
      if (p_saved->name == p_name) {
        p_saved->connected = true;
        found = true;
        break;
      }
      p_saved++;
    }

    // If not in list, add it
    if (!found)
      players->push_back(GamePlayer{p_name, bridge_name + p_name, true});
  }
}

/**════════════════════════════════════════════════════════════════════════
 *?                            Slave mode
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Toggle the slave mode
/// @return true if slave is on
/// @return false if slave is off
bool BridgesManager::slave_toggle() {
  if (slaving) {
    if (slave_off())
      slaving = false;
  } else {
    if (slave_on())
      slaving = true;
  }
  return slaving;
}

/// @brief Active the slave mode for the bridges
/// @return return true if the change was successful, false either case
bool BridgesManager::slave_on() {
  // Reset slave players configuration
  _slave_req.request.right_user = block_player;
  _slave_req.request.left_user = block_player;

  // Set players token to send to the server
  if (_players.expired())
    return slaving;
  auto players = _players.lock();

  auto player = players->begin();
  while (player != players->end()) {
    if (player->side == ArmSide::RIGHT_ARM)
      _slave_req.request.right_user = player->name;
    if (player->side == ArmSide::LEFT_ARM)
      _slave_req.request.left_user = player->name;
    player++;
  }

  // Send service request
  if (send_slave_request())
    slaving = true;
  return slaving;
}

/// @brief Deactivate the slave mode
/// @return return true if the change was successful, false either case
bool BridgesManager::slave_off() {
  _slave_req.request.right_user = free_player;
  _slave_req.request.left_user = free_player;

  // Send service request
  if (send_slave_request())
    slaving = false;

  return slaving;
}

/// @brief Send the slave state to the server
bool BridgesManager::send_slave_request() {
  if (_slave_client != nullptr)
    return _slave_client.call(_slave_req);
  return false;
}

} // namespace ecn_baxter::game::ros1
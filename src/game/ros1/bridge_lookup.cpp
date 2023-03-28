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
#include "ecn_baxter/game/events/auth_refresh_event.hpp"
#include "ecn_baxter/game/events/bridges_update_events.hpp"
#include "ecn_baxter/game/events/event_target.hpp"
#include "ecn_baxter/game/utils/logger.hpp"
#include "ros/duration.h"
#include <chrono>
#include <memory>
#include <qapplication.h>

using namespace std::chrono_literals;

namespace ecn_baxter::game::ros1 {

/**════════════════════════════════════════════════════════════════════════
 *!                           Initialization
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Initialize the periodic bridge lookup related objects
void BridgesManager::bridges_init(std::shared_ptr<ros::NodeHandle> handle,
                                  std::shared_ptr<data::PlayerList> players) {
  _playerlist = players;

  // Services
  _force_client =
      handle->serviceClient<baxter_core_msgs::BridgePublishersForce>(
          force_service);
  _auth_client = handle->serviceClient<baxter_core_msgs::BridgePublishersAuth>(
      auth_service);

  // Timers
  _routine =
      handle->createWallTimer(check_dur, [this](const ros::WallTimerEvent &) {
        look_for_briges();
        if (!_playerlist.expired()) {
          auto pl = _playerlist.lock();
          if (pl->connected > 0) {
            force_routine();
            auth_routine();
          }
        }
        QApplication::sendEvent(events::EventTarget::instance(),
                                new events::BridgesUpdate());
      });
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
  if (!_playerlist.expired()) {
    auto pl = _playerlist.lock();
    auto i = pl->players.begin();
    while (i != pl->players.end()) {
      i->connected = false;
      i++;
    }
  }
}

/// @brief Update local players list to last look out
void BridgesManager::update_connected_players(
    const std::vector<std::string> &to_add) {
  if (_playerlist.expired())
    return;

  auto pl = _playerlist.lock();
  pl->connected = 0;

  bool found;
  for (auto &p_name : to_add) {

    found = false;

    // If players already in list, update to "connected"
    auto p_saved = pl->players.begin();
    while (p_saved != pl->players.end()) {
      if (p_saved->name == p_name) {
        p_saved->connected = true;
        pl->connected++;
        found = true;
        break;
      }
      p_saved++;
    }

    // If not in list, add it
    if (!found)
      pl->players.push_back(GamePlayer{p_name});
  }
}

/**════════════════════════════════════════════════════════════════════════
 *?                            Slave mode
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Active the slave mode for the bridges
/// @return return true if the change was successful, false either case
void BridgesManager::slave_on(bool game_on) { _slaving = true; }

/// @brief Deactivate the slave mode
/// @return return true if the change was successful, false either case
void BridgesManager::slave_off() { _slaving = false; }

void BridgesManager::refresh_force_request() {
  if (_playerlist.expired()) {
    BAXTER_ERROR(PLAYER_LIST_EXPIRED);
    return;
  }

  // Set the authorized users
  if (_slaving) {
    _force_req.request.right_user = GamePlayer::block_player;
    _force_req.request.left_user = GamePlayer::block_player;

    auto pl = _playerlist.lock();

    auto player = pl->players.begin();
    while (player != pl->players.end()) {
      if (player->wanted_side == ArmSide::RIGHT_ARM ||
          player->wanted_side == ArmSide::BOTH)
        _force_req.request.right_user = player->name;
      if (player->wanted_side == ArmSide::LEFT_ARM ||
          player->wanted_side == ArmSide::BOTH)
        _force_req.request.left_user = player->name;
      player++;
    }
  } else {
    _force_req.request.right_user = GamePlayer::free_player;
    _force_req.request.left_user = GamePlayer::free_player;
  }
}

/// @brief Send the slave state to the server
void BridgesManager::force_routine() {
  if (_force_client == nullptr) {
    BAXTER_ERROR(SERVICE_ERROR, PREFIX, "force");
    return;
  }
  if (!_force_client.waitForExistence(_service_timeout)) {
    BAXTER_ERROR(MONITOR_OFFLINE, PREFIX, "force");
    return;
  }

  refresh_force_request();
  _force_client.call(_force_req);
  BAXTER_DEBUG(FORCE_REQ_SENT, PREFIX);
}

/// @brief Get the actual authorized users from the monitor
void BridgesManager::auth_routine() {
  if (_auth_client == nullptr) {
    BAXTER_ERROR(SERVICE_ERROR, PREFIX, "auth");
    return;
  }
  if (!_auth_client.waitForExistence(_service_timeout)) {
    BAXTER_ERROR(MONITOR_OFFLINE, PREFIX, "auth");
    return;
  }

  _auth_client.call(_auth_req);

  if (_playerlist.expired()) {
    BAXTER_ERROR(PLAYER_LIST_EXPIRED);
    return;
  }

  auto p_list = _playerlist.lock();
  p_list->left_user = _auth_req.response.forced_left;
  p_list->right_user = _auth_req.response.forced_right;

  _slaving = p_list->is_slaving();

  BAXTER_DEBUG(AUTH_REQ_SENT, PREFIX);
}

} // namespace ecn_baxter::game::ros1
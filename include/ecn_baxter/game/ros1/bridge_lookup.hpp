/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Sub-part of the ROS1 game master for checking connected
 *                    players
 * @version        :  rev 23w12.4
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef BRIDGE_LOOKUP
#define BRIDGE_LOOKUP

#include "baxter_core_msgs/BridgePublishersAuth.h"
#include "baxter_core_msgs/BridgePublishersForce.h"
#include "ros/service_client.h"
#include "ros/wall_timer.h"
#include <ecn_baxter/game/data/game_players.hpp>
#include <memory>
#include <ros/duration.h>
#include <ros/master.h>
#include <ros/node_handle.h>
#include <vector>

#include <string>
#include <vector>

namespace ecn_baxter::game::ros1 {

using namespace ecn_baxter::game::data;

class BridgesManager {
private:
  static constexpr auto bridge_name{"baxter_ros1_bridge_"};
  static constexpr auto force_service{"/bridge_force"};
  static constexpr auto auth_service{"/bridge_auth"};

  static constexpr auto PLAYER_LIST_EXPIRED{
      "ROS1 Bridge Lookup: Player list expired !"};

  /**════════════════════════════════════════════════════════════════════════
   **                                Utils
   * ════════════════════════════════════════════════════════════════════════**/

  std::string extract_name(const std::string &) const;
  bool is_bridge(const std::string &, std::string &) const;

  /**════════════════════════════════════════════════════════════════════════
   *?                             Bridge Lookup
   * ════════════════════════════════════════════════════════════════════════**/
  std::weak_ptr<data::PlayerList> _playerlist;
  const ros::WallDuration check_dur{0.5};
  ros::WallTimer _routine;
  void update_connected_players(const std::vector<std::string> &);
  void reset_players_state();

  /**════════════════════════════════════════════════════════════════════════
   *?                              Slave mode
   * ════════════════════════════════════════════════════════════════════════**/
  static constexpr auto PREFIX{"Auth: "};
  static constexpr auto SERVICE_ERROR{
      "%s %s service wasn't initialized correctly ! Please report it to "
      "developpers !"};
  static constexpr auto MONITOR_OFFLINE{
      "%s monitor (%s) isn't online ! Please start the baxter_bridge !"};
  static constexpr auto FORCE_REQ_SENT{
      "%s authorized users sent to the monitor."};
  static constexpr auto AUTH_REQ_SENT{
      "%s autorized users list asked to the monitor."};

  const ros::Duration _service_timeout{.5};
  ros::ServiceClient _force_client, _auth_client;
  baxter_core_msgs::BridgePublishersForce _force_req;
  baxter_core_msgs::BridgePublishersAuth _auth_req;
  bool slaving = false;
  void refresh_force_request();
  void force_routine();
  void auth_routine();

protected:
  /**════════════════════════════════════════════════════════════════════════
   *!                             Initialization
   * ════════════════════════════════════════════════════════════════════════**/

  explicit BridgesManager(){};
  void bridges_init(std::shared_ptr<ros::NodeHandle>,
                    std::shared_ptr<data::PlayerList>);

  /**════════════════════════════════════════════════════════════════════════
   *?                              Bridge Lookup
   * ════════════════════════════════════════════════════════════════════════**/

  void look_for_briges();

public:
  /**════════════════════════════════════════════════════════════════════════
   *?                              Slave mode
   * ════════════════════════════════════════════════════════════════════════**/

  void slave_on(bool = false);
  void slave_off();
  /// @brief Return whether the bridges are in slave mode or not
  inline bool is_slaving() { return slaving; };
};
} // namespace ecn_baxter::game::ros1

#endif
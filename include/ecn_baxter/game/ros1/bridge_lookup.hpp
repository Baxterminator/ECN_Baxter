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

#include "baxter_core_msgs/BridgePublishersForce.h"
#include "ros/service_client.h"
#include <ecn_baxter/game/data/game_players.hpp>
#include <ecn_baxter/utils.hpp>
#include <ros/duration.h>
#include <ros/master.h>
#include <ros/node_handle.h>
#include <vector>

#include <sstream>
#include <string>
#include <vector>

namespace ecn_baxter::game::ros1 {

using namespace ecn_baxter::game::data;

class BridgesManager {
private:
  static constexpr auto bridge_name{"baxter_ros1_bridge_"};
  static constexpr auto slave_service{"/bridge_force"};

  /**========================================================================
   **                             Utils
   *========================================================================**/
  std::string extract_name(const std::string &) const;
  bool is_bridge(const std::string &, std::string &) const;

  /**========================================================================
   **                            Bridge Lookup
   *========================================================================**/
  const ros::WallDuration check_dur{0.5};
  ros::WallTimer _check_timer;
  std::function<void(PlayerList &)> _callback;
  void update_connected_players(const std::vector<std::string> &);
  void reset_players_state();

  /**========================================================================
   **                            Slave mode
   *========================================================================**/
  ros::ServiceClient _slave_client;
  baxter_core_msgs::BridgePublishersForce _slave_req;
  bool slaving = false;
  bool send_slave_request();

protected:
  /**========================================================================
   *!                           Initialization
   *========================================================================**/
  explicit BridgesManager(){};
  void bridges_init(sptr<ros::NodeHandle>);
  PlayerList players;

  /**========================================================================
   **                            Bridge Lookup
   *========================================================================**/
  void look_for_briges();

public:
  /**========================================================================
   **                            Bridge Lookup
   *========================================================================**/
  inline void set_look_up_callback(std::function<void(PlayerList &)> callback) {
    _callback = callback;
  }

  /**========================================================================
   **                            Slave mode
   *========================================================================**/
  bool slave_toggle();
  bool slave_on();
  bool slave_off();
};
} // namespace ecn_baxter::game::ros1

#endif
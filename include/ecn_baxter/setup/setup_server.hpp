/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  26/02/2023
 * @description    :  Setup server for initializing all game-related resources
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_SETUP_NODE_HPP
#define ECN_BAXTER_SETUP_NODE_HPP

#include "ecn_baxter/setup/point_server.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ecn_baxter::setup {

/// @brief Setup server node
class SetupServer {
public:
  explicit SetupServer(rclcpp::NodeOptions);

  std::shared_ptr<rclcpp::Node> getNode() { return node; }

protected:
  static constexpr auto node_name{"setup_server"};
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<PointServer> ptn_server;
};

} // namespace ecn_baxter::setup

#endif

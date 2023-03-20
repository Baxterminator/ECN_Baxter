/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  26/02/2023
 * @description    :  Setup server for initializing all game-related resources
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/setup/setup_server.hpp"
#include "ecn_baxter/game/data/arm_side.hpp"
#include "ecn_baxter/setup/point_server.hpp"
#include <functional>
#include <memory>
#include <rclcpp/node_options.hpp>
#include <thread>

namespace ecn_baxter::setup {

SetupServer::SetupServer(rclcpp::NodeOptions opts) {
  node = std::make_shared<rclcpp::Node>(node_name, opts);
  ptn_server = std::make_shared<PointServer>(node);
}

} // namespace ecn_baxter::setup

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ecn_baxter::setup::SetupServer server(rclcpp::NodeOptions{});
  rclcpp::spin(server.getNode());
  rclcpp::shutdown();
  return 0;
}
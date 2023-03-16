#ifndef PROJ1_SETUP_NODE_HPP
#define PROJ1_SETUP_NODE_HPP

#include "ecn_baxter/setup/point_server.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ecn_baxter::setup {

class SetupServer {
public:
  explicit SetupServer(rclcpp::NodeOptions);

  sptr<rclcpp::Node> getNode() { return node; }

protected:
  static constexpr auto node_name{"setup_server"};
  sptr<rclcpp::Node> node;
  sptr<PointServer> ptn_server;
};
} // namespace ecn_baxter::setup

#endif // PROJ1_SETUP_NODE_HPP

//
// Created by tcorroenne2021 on 08/02/23.
// Modified by B on 08/03/23.
//  <depend>kdl_parser</depend>

#include "rclcpp/rclcpp.hpp"
#include <Eigen/QR>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ecn_baxter/srv/jacobian.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <urdf/model.h>

using ecn_baxter::srv::Jacobian;
using JacReq = Jacobian::Request::SharedPtr;
using JacRes = Jacobian::Response::SharedPtr;

struct Solvers {
  explicit Solvers(const KDL::Chain &chain);
  KDL::ChainFkSolverPos_recursive fwd;
  KDL::ChainIkSolverVel_pinv ik_v;
  KDL::ChainIkSolverPos_NR ik_p;
  KDL::ChainJntToJacSolver jac;
};

Solvers::Solvers(const KDL::Chain &chain)
    : fwd{chain}, ik_v{chain}, ik_p{chain, fwd, ik_v}, jac{chain} {}

std::unique_ptr<urdf::Model> initRSP() {
  const auto baxter_folder =
      ament_index_cpp::get_package_share_directory("baxter_description");
  const auto description_file{baxter_folder + "/urdf/baxter.urdf.xacro"};

  // yes we still have to process a command output in 2022
  FILE *stream;
  const int max_buffer = 256;
  std::string cmd{"xacro "};
  cmd += description_file;
  stream = popen(cmd.c_str(), "r");
  std::string xml;

  if (stream) {
    while (!feof(stream)) {
      char buffer[max_buffer];
      if (fgets(buffer, max_buffer, stream) != NULL)
        xml.append(buffer);
    }
    pclose(stream);
  }

  auto model{std::make_unique<urdf::Model>()};
  model->initString(xml);
  return model;
}

namespace ecn_baxter::utils {
class JacobianNode : public rclcpp::Node {
public:
  JacobianNode(rclcpp::NodeOptions options) : Node("node_name", options) {
    const auto model{initRSP()};
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(*model, tree);
    tree.getChain("base", side + "_gripper", arm_chain);
    solvers = std::make_unique<Solvers>(arm_chain);
    jacobian_service = this->create_service<Jacobian>(
        "/robot/limb/" + side + "/jacobian",
        [&](JacReq req, JacRes res) { Jacobian_calc(req, res); });
  }

private:
  // declare any subscriber / publisher / timer
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber;
  geometry_msgs::msg::Twist last_twist;
  rclcpp::Service<Jacobian>::SharedPtr jacobian_service;

  std::string side = "left";
  KDL::Chain arm_chain;
  std::unique_ptr<Solvers> solvers;

  void Jacobian_calc(const JacReq req,
                     JacRes res) // taken from baxter_simple_sim
  {
    KDL::JntArray q(7);
    std::copy(req->position.begin(), req->position.end(), q.data.data());
    // get base Jacobian fJe
    KDL::Jacobian J(7);
    solvers->jac.JntToJac(q, J);

    if (req->ee_frame) {
      // we want eJe, rotate
      KDL::Frame fMe;
      solvers->fwd.JntToCart(q, fMe);
      J.changeBase(fMe.M.Inverse());
    }

    const auto writeMatrix =
        [res](const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &M) {
          auto elem{res->jacobian.begin()};
          for (int row = 0; row < M.rows(); ++row) {
            for (int col = 0; col < M.cols(); ++col)
              *elem++ = M(row, col);
          }
        };
    if (req->inverse)
      writeMatrix(J.data.completeOrthogonalDecomposition().pseudoInverse());
    else
      writeMatrix(J.data);
  }
};

} // namespace ecn_baxter::utils

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<ecn_baxter::utils::JacobianNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

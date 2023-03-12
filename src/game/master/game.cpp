/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Main class for game management and logic
 *========================================================================**/
#include "ecn_baxter/game/data/game_players.hpp"
#include <ecn_baxter/game/master/game.hpp>
#include <iostream>
#include <qpushbutton.h>
#include <thread>

namespace ecn_baxter::game {
std::thread Game::ros_thread;
sig_atomic_t Game::stop_cmd;
sptr<Game> Game::_instance;

/**========================================================================
 *!                           INITIALIZATION
 *========================================================================**/
Game::Game() : GamePropertiesLoader(), UIManager() { stop_cmd = 0; }

/// @brief Initialize everything related to the ROS 1&2 nodes
/// @return true if the initialization as successful
bool Game::init(int argc, char **argv) {

  //* ROS2 Initialization
  rclcpp::init(argc, argv);
  ros2_node = std::make_shared<ros2::GameMaster_2>(rclcpp::NodeOptions{});

  ex = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ex->add_node(ros2_node);

  //* ROS1 Initialization
  ros::init(argc, argv, ros1::GameMaster_1::NODE_NAME);

  /*
          // Checking if another game master is launched
          vector<string> nodes; ros::master::getNodes(nodes);
          auto it = find(nodes.begin(), nodes.end(), "game_master_1");
          if (it != nodes.end()) {
              RCLCPP_FATAL(ros2()->get_logger(), "Game master is already
     launched !"); return false;
          }*/
  ros1_node = std::make_shared<ros1::GameMaster_1>();

  //* Run ROS Loop
  ros_thread = std::thread(&Game::ros_loop);

  return true;
}

/**========================================================================
 *?                           Game Management
 *========================================================================**/
/// @brief Load the game parameters from a JSON file
/// @param file_path the path to the configuration file
void Game::load_game_propeties(const std::string &file_path) {
  RCLCPP_INFO(ros2_node->get_logger(), "Loading game %s", file_path.c_str());
  load_file(file_path);
  main_window->get_ui()->game_name->setText(game_props->game_name.c_str());
  if (game_props->is_skippable()) {
    main_window->get_ui()->setup->setEnabled(false);
    main_window->get_ui()->launch->setEnabled(true);
  } else {
    main_window->get_ui()->setup->setEnabled(true);
    main_window->get_ui()->launch->setEnabled(false);
  }
}

/**========================================================================
 **                            Cycle Utils
 *========================================================================**/
/// @brief Return the current instance of the Game
sptr<Game> Game::instance() {
  if (_instance == nullptr)
    _instance = std::make_shared<Game>();
  return _instance;
}

bool Game::Init(int argc, char **argv) { return instance()->init(argc, argv); }

/// @brief Run one cycle of the ROS nodes
void Game::spinOnce() { instance()->ex->spin_once(10ms); }

/// @brief Send a stop signal to all ROS-related thread
void Game::stop() {
  stop_cmd = 1;
  if (ros_thread.joinable())
    ros_thread.join();
  ros::shutdown();
  rclcpp::shutdown();
  clean();
}

/// @brief Cleaning procedure for erasing all left pointers that could be left
void Game::clean() {
  game_props.reset();
  _instance.reset();
}

/**========================================================================
 **                            Game Thread
 *========================================================================**/
/// @brief Main task for ros loop
void Game::ros_loop() {
  ros::AsyncSpinner async(1);
  async.start();
  while (ros::ok() && rclcpp::ok() && !stop_cmd) {
    Game::spinOnce();
  }
}

/**========================================================================
 **                            UI Management
 *========================================================================**/
/// @brief Show the main GUI, make events binding, etc
void Game::showUI() {
  instance()->show_ui();
  instance()->ros1_node->set_look_up_callback([&](data::PlayerList &list) {
    instance()->main_window->refresh_player_list(list);
  });
}

/// @brief Set all bindings between GUI and Game Background tasks
void Game::_bind_ui() {
  //* Game Loader bindings
  QObject::connect(main_window->get_game_loader()->get_ui()->select_buttons,
                   &QDialogButtonBox::accepted, [&]() {
                     load_game_propeties(
                         main_window->get_game_loader()->get_file_to_load());
                   });
  QObject::connect(main_window->get_ui()->setup, &QPushButton::clicked,
                   [&]() { ros2_node->launch_point_setup(game_props); });

  //* Slave mode bindings
  QObject::connect(main_window->get_ui()->left_arm_token, &QPushButton::clicked,
                   [&]() {
                     main_window->get_ui()->slave->setEnabled(false);
                     main_window->get_ui()->slave_on->setText(
                         (ros1_node->slave_toggle()) ? "ON" : "OFF");
                     main_window->get_ui()->slave->setEnabled(true);
                   });
}
} // namespace ecn_baxter::game
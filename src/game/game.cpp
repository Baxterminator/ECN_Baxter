/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Main class for game management and logic
 *========================================================================**/
#include <ecn_baxter/game/game.hpp>

namespace ecn_baxter::game {
sptr<ros1::GameMaster_1> Game::ros1_node;
sptr<GameMaster_2> Game::ros2_node;
sptr<rclcpp::executors::SingleThreadedExecutor> Game::ex;
sptr<gui::MainUI> Game::main_window;
std::thread Game::ros_thread;
sig_atomic_t Game::stop_cmd;

sptr<std::function<void()>> Game::bindings;

/**========================================================================
 *!                           INITIALIZATION
 *========================================================================**/
/// @brief Initialize everything related to the ROS 1&2 nodes
/// @return true if the initialization as successful
bool Game::init(int argc, char **argv) {
  //* Cycle initialization
  stop_cmd = 0;

  //* ROS2 Initialization
  rclcpp::init(argc, argv);
  ros2_node = std::make_shared<GameMaster_2>(rclcpp::NodeOptions{});

  ex = rclcpp::executors::SingleThreadedExecutor::make_shared();
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

  //* UI Initialisation
  bindings = std::make_shared<std::function<void()>>();
  *bindings = []() { Game::bind_gui(); };

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

void Game::show_gui() {
  main_window = std::make_shared<gui::MainUI>(bindings);
  main_window->setup_ui();
  main_window->show();
}

/// @brief Set all bindings between GUI and Game Background tasks
void Game::_bind_gui() {
  QObject::connect(main_window->get_game_loader()->get_ui()->select_buttons,
                   &QDialogButtonBox::accepted, [&]() {
                     Game::load_game_propeties(
                         main_window->get_game_loader()->get_file_to_load());
                   });
  QObject::connect(main_window->get_ui()->setup, &QPushButton::clicked,
                   [&]() { Game::ros2()->launch_point_setup(game_props); });
}
} // namespace ecn_baxter::game
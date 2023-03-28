/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Main wrapper managing ROS threads as well as UIs
 * @version        :  rev 23w12.2
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/master/game.hpp"
#include "ecn_baxter/game/data/game_players.hpp"
#include "ecn_baxter/game/events/auth_refresh_event.hpp"
#include "ecn_baxter/game/events/bridges_update_events.hpp"
#include "ecn_baxter/game/events/event_target.hpp"
#include "ecn_baxter/game/events/log_event.hpp"
#include "ecn_baxter/game/events/setup_ended.hpp"
#include "ecn_baxter/game/ros1/bridge_lookup.hpp"
#include "ecn_baxter/game/utils/logger.hpp"
#include "ecn_baxter/game/utils/qtevents.hpp"
#include <baxter_bridge/monitor.h>
#include <memory>
#include <qcoreevent.h>
#include <qglobal.h>
#include <qobject.h>
#include <qpushbutton.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>

namespace ecn_baxter::game {
sig_atomic_t Game::stop_cmd;

/**═════════════════════════════════════════════════════════════════════════
 *!                           INITIALIZATION
 * ═════════════════════════════════════════════════════════════════════════*/

/// @brief Initialize the application and every ROS / GUI components
Game::Game(int &argc, char **argv)
    : QApplication(argc, argv), GamePropertiesLoader() {
  stop_cmd = 0;
  players_list = std::make_shared<data::PlayerList>();

  //* ROS2 Initialization
  rclcpp::init(argc, argv);
  ros2_node = std::make_shared<ros2::GameMaster_2>(rclcpp::NodeOptions{});
  ros2_node->set_handle(ros2_node);

  ex = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  ex->add_node(ros2_node);

  //* ROS1 Initialization
  ros::init(argc, argv, ros1::GameMaster_1::NODE_NAME);

  // TODO: Checkup if another game master is on
  /*
          // Checking if another game master is launched
          vector<string> nodes; ros::master::getNodes(nodes);
          auto it = find(nodes.begin(), nodes.end(), "game_master_1");
          if (it != nodes.end()) {
              BAXTER_FATAL("Game master is already
     launched !"); return false;
          }*/
  ros1_node = std::make_unique<ros1::GameMaster_1>(players_list);

  //* Run ROS Loop
  ros_thread = std::thread([&]() { ros_loop(); });

  //* All inits have been made successfully
  _initialized = true;
}

/**═════════════════════════════════════════════════════════════════════════
 *!                            Game Thread
 * ═════════════════════════════════════════════════════════════════════════*/

/// @brief Main task for ROS1/ROS2 loop
void Game::ros_loop() const {
  ros::AsyncSpinner async(1);
  async.start();
  while (ros::ok() && rclcpp::ok() && !stop_cmd) {
    ex->spin_once(10ms);
  }
}

/// @brief Send a stop signal to all ROS-related thread
void Game::stop() {
  stop_cmd = 1;
  if (ros_thread.joinable())
    ros_thread.join();
  ros::shutdown();
  rclcpp::shutdown();
}

/**═════════════════════════════════════════════════════════════════════════
 *?                           Game Management
 * ═════════════════════════════════════════════════════════════════════════*/

/// @brief Load the game parameters from a JSON file
/// @param file_path the path to the configuration file
void Game::load_game_propeties(const std::string &file_path) {
  BAXTER_INFO("Loading game %s", file_path.c_str());
  load_file(file_path);
  main_ui->get_ui()->game_name->setText(game_props->game_name.c_str());
  if (game_props->is_skippable()) {
    main_ui->get_ui()->game_setup->setEnabled(false);
    main_ui->get_ui()->game_idling->setEnabled(true);
  } else {
    main_ui->get_ui()->game_setup->setEnabled(true);
    main_ui->get_ui()->game_idling->setEnabled(false);
  }

  // Update game properties pointers
  ros1_node->update_game_props(game_props);
  ros2_node->update_game_props(game_props);
}

/**═════════════════════════════════════════════════════════════════════════
 **                            UI Management
 * ═════════════════════════════════════════════════════════════════════════*/

/// @brief Show the main GUI, make events binding, etc
void Game::show_ui() {
  if (main_ui != nullptr)
    return;

  main_ui = std::make_unique<gui::MainUI>();
  main_ui->setup_ui();
  main_ui->set_player_list(players_list);
  main_ui->show();
}

// Preprocessor directives for easier reading and writings
#define event_is(t) (ev->type() == t)
#define is_null(t) (t == nullptr)
#define receiver_is(t) (receiver == t)
#define ui main_ui->get_ui()
#define game_loader main_ui->get_game_loader()
#define press_event event_is(QEvent::MouseButtonRelease)

/// @brief Logic between UI & Game works
bool Game::notify(QObject *receiver, QEvent *ev) {
  //! ━━━━━━━━━━━━━━━━━━━━ Non-UI components callbacks ━━━━━━━━━━━━━━━━━━━━━*/
  if (!event_is(QEvent::Create) &&
      receiver_is(events::EventTarget::instance())) {
    if (slave_logic(receiver, ev, false) || bridge_logic(receiver, ev, false) ||
        setup_logic(receiver, ev, false) || game_logic(receiver, ev, false)) {
      return false;
    } else if (event_is(events::LogEvent::type())) {
      return true;
    }
    //*══════════════════ ELSE (Without useless warnings) ═════════════════*/
    else if (!event_is(QEvent::Polish) && !event_is(QEvent::PolishRequest) &&
             !event_is(QEvent::None)) {
      BAXTER_WARN("Unknown custom callback from non UI component (%s %d)!",
                  utils::qt::get_qtevent_name(ev).c_str(), ev->type());
    }
  }

  //! ━━━━━━━━━━━━━━━━━━━━━━━━━━━━ UI callbacks ━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
  if (!is_null(main_ui) && main_ui->is_made()) {
    if (slave_logic(receiver, ev, true) || bridge_logic(receiver, ev, true) ||
        setup_logic(receiver, ev, true) || game_logic(receiver, ev, true)) {
      return false;
    }
  }

  return QApplication::notify(receiver, ev);
}

/// @brief UI logic for game management
bool Game::bridge_logic([[maybe_unused]] QObject *receiver, QEvent *ev,
                        bool is_from_ui) {
  //! ━━━━━━━━━━━━━━━━━━━━━━━━━━━━ UI callbacks ━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
  if (is_from_ui) {

  }
  //! ━━━━━━━━━━━━━━━━━━━━ Non-UI components callbacks ━━━━━━━━━━━━━━━━━━━━━*/
  else {
    //*═══════════════════════════ Bridge list ════════════════════════════*/
    if ((event_is(events::BridgesUpdate::type()) ||
         event_is(events::AuthRefresh::type())) &&
        !is_null(main_ui) && main_ui->is_made() && !is_null(ros2_node)) {
      BAXTER_DEBUG(
          "Updating bridges list (%zu registered bridges).",
          ((players_list != nullptr) ? players_list->players.size() : 0));
      main_ui->refresh_player_list(ros1_node->is_slaving(),
                                   ros1_node->get_slave_mode() ==
                                       ros1::SlaveMode::Block_All);
      if (!game_idle && !game_launched) {
        ui->slave_on->setEnabled(true);
        ui->slave_mode->setEnabled(true);
      }
      return true;
    }
  }
  return false;
}

/// @brief UI logic for game management
bool Game::setup_logic(QObject *receiver, QEvent *ev, bool is_from_ui) {
  //! ━━━━━━━━━━━━━━━━━━━━━━━━━━━━ UI callbacks ━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
  if (is_from_ui) {
    //*═════════════════════════ Launching setup ═══════════════════════════*/
    if (receiver_is(ui->game_setup) && press_event) {
      //? Setup launching
      ui->game_setup->setEnabled(false);
      ui->game_load->setEnabled(false);
      if (!ros2_node->make_setup()) {
        ui->game_setup->setEnabled(true);
        ui->game_load->setEnabled(true);
      }

      return true;
    }
  }
  //! ━━━━━━━━━━━━━━━━━━━━ Non-UI components callbacks ━━━━━━━━━━━━━━━━━━━━━*/
  else {
    //*═══════════════════════════ Setup ended ═════════════════════════════*/
    if (event_is(events::SetupEnded::type())) {
      ui->game_idling->setEnabled(true);
      ui->game_setup->setEnabled(true);
      ui->game_load->setEnabled(true);
      return true;
    }
  }
  return false;
}

/// @brief UI logic for game management
bool Game::game_logic(QObject *receiver, QEvent *ev, bool is_from_ui) {
  //! ━━━━━━━━━━━━━━━━━━━━━━━━━━━━ UI callbacks ━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
  if (is_from_ui) {
    //*══════════════════════════  Game choosing ═══════════════════════════*/
    if (!is_null(game_loader) && game_loader->is_made() &&
        receiver_is(game_loader->get_ui()->select_buttons->buttons().at(0)) &&
        press_event) {
      load_game_propeties(game_loader->get_file_to_load());
      return false;
    }
    //*═══════════════════════════ Idling mode ═════════════════════════════*/
    else if (receiver_is(ui->game_idling) && press_event) {
      if (game_idle) {
        ros1_node->slave_on();
        ros1_node->set_slave_mode(ros1::SlaveMode::Block_All);
        ui->game_idling->setText("Start Game Lobby");
        ui->slave_on->setEnabled(true);
        ui->slave_mode->setEnabled(true);
        ui->game_setup->setEnabled(true);
        ui->game_load->setEnabled(true);
        ui->game_launch->setEnabled(false);
        game_idle = false;
      } else {
        ros1_node->slave_on();
        ros1_node->set_slave_mode(ros1::SlaveMode::Block_All);
        ui->game_idling->setText("End Game Lobby");
        ui->slave_on->setEnabled(false);
        ui->slave_mode->setEnabled(false);
        ui->game_setup->setEnabled(false);
        ui->game_load->setEnabled(false);
        ui->game_launch->setEnabled(true);
        game_idle = true;
      }
      return true;
    }
    //*════════════════════════ Launch / Stop Game ═════════════════════════*/
    else if (receiver_is(ui->game_launch) && press_event) {
      // Whether to start or stop the game
      if (game_launched) {
        game_launched = false;
        ui->game_launch->setText("Start Game");
        ui->game_idling->setEnabled(true);
        ros1_node->slave_on();
        ros1_node->set_slave_mode(ros1::SlaveMode::Block_All);
      } else {
        game_launched = true;
        ui->game_launch->setText("Stop Game");
        ui->game_idling->setEnabled(false);
        ros1_node->slave_on();
        ros1_node->set_slave_mode(ros1::SlaveMode::Selected_Player);
      }
      return true;
    }
  }
  //! ━━━━━━━━━━━━━━━━━━━━ Non-UI components callbacks ━━━━━━━━━━━━━━━━━━━━━*/
  else {
  }
  return false;
}

/// @brief UI logic for bridge slaving
bool Game::slave_logic(QObject *receiver, QEvent *ev, bool is_from_ui) {
  //! ━━━━━━━━━━━━━━━━━━━━━━━━━━━━ UI callbacks ━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
  if (is_from_ui) {
    //*══════════════════════════ Slave On/Off ═════════════════════════════*/
    if (receiver_is(ui->slave_on) && press_event) {
      ui->slave_on->setEnabled(false);
      // Set slave to ON or OFF
      (game_launched || !ros1_node->is_slaving()) ? ros1_node->slave_on()
                                                  : ros1_node->slave_off();
      return true;
    }
    //*══════════════════════════ Slave Mode ═════════════════════════════*/
    else if (receiver_is(ui->slave_mode) && press_event) {
      ui->slave_mode->setEnabled(false);
      ros1_node->set_slave_mode(
          (ros1_node->get_slave_mode() == ros1::SlaveMode::Block_All)
              ? ros1::SlaveMode::Selected_Player
              : ros1::SlaveMode::Block_All);
      return true;
    }
  }
  //! ━━━━━━━━━━━━━━━━━━━━ Non-UI components callbacks ━━━━━━━━━━━━━━━━━━━━━*/
  else {
  }
  return false;
}
} // namespace ecn_baxter::game
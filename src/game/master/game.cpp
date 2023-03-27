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
#include "ecn_baxter/game/utils/logger.hpp"
#include "ecn_baxter/game/utils/qtevents.hpp"
#include <memory>
#include <qcoreevent.h>
#include <qglobal.h>
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
    main_ui->get_ui()->setup->setEnabled(false);
    main_ui->get_ui()->launch->setEnabled(true);
  } else {
    main_ui->get_ui()->setup->setEnabled(true);
    main_ui->get_ui()->launch->setEnabled(false);
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

/// @brief Logic between UI & Game works
bool Game::notify(QObject *receiver, QEvent *ev) {
  using namespace events;

  /*std::cout << "Get event " << utils::qt::get_qtevent_name(ev) << " for "
            << ((receiver == nullptr) ? "null"
                                      : receiver->objectName().toStdString())
  << std::endl;*/

  // Preprocessor directives for easier reading and writings
#define event_is(t) (ev->type() == t)
#define is_null(t) (t == nullptr)
#define receiver_is(t) (receiver == t)

  //! ━━━━━━━━━━━━━━━━━━━━ Non-UI components callbacks ━━━━━━━━━━━━━━━━━━━━━*/
  if (!event_is(QEvent::Create) && receiver_is(EventTarget::instance())) {
    //*═══════════════════════════ BRIDGE LIST ════════════════════════════*/
    if ((event_is(BridgesUpdate::type()) || event_is(AuthRefresh::type())) &&
        !is_null(main_ui) && main_ui->is_made() && !is_null(ros2_node)) {
      BAXTER_DEBUG(
          "Updating bridges list (%zu registered bridges).",
          ((players_list != nullptr) ? players_list->players.size() : 0));
      main_ui->refresh_player_list();
    }
    //*═══════════════════════════ SETUP ENDED ════════════════════════════*/
    else if (event_is(SetupEnded::type())) {
      main_ui->get_ui()->launch->setEnabled(true);
    }
    //*═════════════════════════════ LOGGING ═════════════════════════════*/
    else if (event_is(LogEvent::type())) {
    }
    //*══════════════════ ELSE (Without useless warnings) ═════════════════*/
    else if (!event_is(QEvent::Polish) && !event_is(QEvent::PolishRequest))
      BAXTER_WARN("Unknown custom callback from non UI component (%s) !",
                  utils::qt::get_qtevent_name(ev).c_str());
  }

  //! ━━━━━━━━━━━━━━━━━━━━━━━━━━━━ UI callbacks ━━━━━━━━━━━━━━━━━━━━━━━━━━━━*/
  if (!is_null(main_ui) && main_ui->is_made()) {
    //*═══════════════════════════ SLAVE MODE ═════════════════════════════*/
    if (receiver_is(main_ui->get_ui()->slave) &&
        event_is(QEvent::MouseButtonRelease)) {
      main_ui->get_ui()->slave->setEnabled(false);
      // Set slave to ON or OFF
      (game_launched || !ros1_node->is_slaving())
          ? ros1_node->slave_on(game_launched)
          : ros1_node->slave_off();
      main_ui->get_ui()->slave->setEnabled(true);
    }
    //*══════════════════════════  SETUP PHASE ════════════════════════════*/
    else if (receiver_is(main_ui->get_ui()->setup) &&
             event_is(QEvent::MouseButtonRelease)) {
      //? Setup launching
      main_ui->get_ui()->setup->setEnabled(false);
      if (!ros2_node->make_setup())
        main_ui->get_ui()->setup->setEnabled(true);

    }
    //*══════════════════════════  GAME LOADING ═══════════════════════════*/
    else if (!is_null(main_ui->get_game_loader()) &&
             main_ui->get_game_loader()->is_made() &&
             receiver_is(main_ui->get_game_loader()
                             ->get_ui()
                             ->select_buttons->buttons()
                             .at(0)) &&
             event_is(QEvent::MouseButtonRelease)) {
      load_game_propeties(main_ui->get_game_loader()->get_file_to_load());
    }
  }

  return QApplication::notify(receiver, ev);
}
} // namespace ecn_baxter::game
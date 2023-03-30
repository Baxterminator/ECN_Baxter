/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Main wrapper managing ROS threads as well as UIs logic
 * @version        :  rev 23w12.2
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_GAME_HPP
#define ECN_BAXTER_GAME_HPP

#include "ecn_baxter/game/data/game_players.hpp"
#include "ecn_baxter/game/properties_loader.hpp"
#include "ecn_baxter/game/ros1/game_master_1.hpp"
#include "ecn_baxter/game/ros2/game_master_2.hpp"
#include "ecn_baxter/game/ui/main_wrapper.hpp"
#include "ros/node_handle.h"
#include <chrono>
#include <memory>
#include <qapplication.h>
#include <qcoreevent.h>
#include <qobject.h>
#include <rclcpp/executors.hpp>
#include <thread>

namespace ecn_baxter::game {
using namespace std::chrono_literals;

/// @brief Main wrapper managing ROS threads as well as UIs logic
class Game : public QApplication, GamePropertiesLoader {
public:
  /**═════════════════════════════════════════════════════════════════════════
   *!                              Cycle Utils
   * ═════════════════════════════════════════════════════════════════════════*/

  Game(int &argc, char **argv);
  inline bool is_initialized() { return _initialized; }
  void stop();

  /**═════════════════════════════════════════════════════════════════════════
  **                                UI Section
  * ═════════════════════════════════════════════════════════════════════════*/

  void show_ui();

  rclcpp::Node *ros2() { return ros2_node.get(); }

private:
  bool _initialized = false;

  /**═════════════════════════════════════════════════════════════════════════
   *?                            ROSX Game Thread
   * ═════════════════════════════════════════════════════════════════════════*/

  std::unique_ptr<ros1::GameMaster_1> ros1_node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ex;
  std::shared_ptr<ros2::GameMaster_2> ros2_node;
  std::thread ros_thread;

  static sig_atomic_t stop_cmd;

  void ros_loop() const;

  /**═════════════════════════════════════════════════════════════════════════
   *?                           Game Management
   * ═════════════════════════════════════════════════════════════════════════*/

  bool game_launched = false, game_idle = false;
  std::shared_ptr<data::PlayerList> players_list;
  void load_game_propeties(const std::string &);

  /**═════════════════════════════════════════════════════════════════════════
   **                              UI Section
   * ═════════════════════════════════════════════════════════════════════════*/

  std::unique_ptr<gui::MainUI> main_ui = nullptr;
  bool notify(QObject *, QEvent *) override;

  bool bridge_logic(QObject *, QEvent *, bool);
  bool game_logic(QObject *, QEvent *, bool);
  bool setup_logic(QObject *, QEvent *, bool);
  bool slave_logic(QObject *, QEvent *, bool);
};
} // namespace ecn_baxter::game

#endif

/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Main wrapper managing ROS threads as well as UIs
 *========================================================================**/

#ifndef GAME_HPP
#define GAME_HPP

#include "ecn_baxter/game/properties_loader.hpp"
#include "ecn_baxter/game/ros1/game_master_1.hpp"
#include "ecn_baxter/game/ros2/game_master_2.hpp"
#include "ecn_baxter/game/ui/main_wrapper.hpp"
#include "ecn_baxter/utils.hpp"
#include "ros/init.h"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <qapplication.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <ros/master.h>
#include <thread>

namespace ecn_baxter::game {
using namespace std::chrono_literals;

/// @brief Main class for game management and logic
class Game : public QApplication, GamePropertiesLoader {
public:
  /**========================================================================
   **                            Cycle Utils
   *========================================================================**/
  Game(int argc, char **argv);
  bool is_initialized() { return _initialized; }
  void stop();

  /**========================================================================
  **                            UI Management
  *========================================================================**/
  void show_ui();

private:
  bool _initialized = false;

  /**========================================================================
   *?                            ROSX Game Thread
   *========================================================================**/
  std::unique_ptr<ros1::GameMaster_1> ros1_node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> ex;
  std::shared_ptr<ros2::GameMaster_2> ros2_node;
  std::thread ros_thread;

  static sig_atomic_t stop_cmd;

  void ros_loop() const;

  /**========================================================================
   *?                           Game Management
   *========================================================================**/
  void load_game_propeties(const std::string &);

  /**========================================================================
   **                              UI Section
   *========================================================================**/
  std::unique_ptr<gui::MainUI> main_ui = nullptr;
  void _bind_ui();

  bool notify(QObject *, QEvent *) override;
};
} // namespace ecn_baxter::game

#endif // GAME_HPP

/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Main class for game management and logic
 *========================================================================**/

#ifndef GAME_HPP
#define GAME_HPP

#include "ros/init.h"
#include <chrono>
#include <cstdlib>
#include <ecn_baxter/game/master/gui_management.hpp>
#include <ecn_baxter/game/properties_loader.hpp>
#include <ecn_baxter/game/ros1/game_master_1.hpp>
#include <ecn_baxter/game/ros2/game_master_2.hpp>
#include <ecn_baxter/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <ros/master.h>
#include <thread>

namespace ecn_baxter::game {
using namespace std::chrono_literals;

/// @brief Main class for game management and logic
class Game : GamePropertiesLoader, UIManager {
private:
  /**========================================================================
   *!                           SINGLETON PATTERN
   *========================================================================**/
  static sptr<Game> _instance;
  static sptr<Game> instance();

public:
  /**========================================================================
   **                            Cycle Utils
   *========================================================================**/
  static bool Init(int argc, char **argv);
  static void spinOnce();
  static void stop();
  static void clean();

  Game();
  ~Game() {
    ros1_node.reset();
    ros2_node.reset();
    ex.reset();
  }

  /**========================================================================
  **                            UI Management
  *========================================================================**/
  static void showUI();

protected:
  /**========================================================================
   *?                            ROS Node
   *========================================================================**/
  sptr<ros1::GameMaster_1> ros1_node;
  sptr<rclcpp::executors::SingleThreadedExecutor> ex;
  sptr<ros2::GameMaster_2> ros2_node;

  /**========================================================================
   *?                           Game Management
   *========================================================================**/
  void load_game_propeties(const std::string &);

  /**========================================================================
   **                            UI Management
   *========================================================================**/
  void _bind_ui() override;

  /**========================================================================
   **                            Game Thread
   *========================================================================**/
  static std::thread ros_thread;
  static void ros_loop();
  static sig_atomic_t stop_cmd;

  bool init(int argc, char **argv);
};
} // namespace ecn_baxter::game

#endif // GAME_HPP

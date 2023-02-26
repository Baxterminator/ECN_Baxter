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
#include <ecn_baxter/game/game_master_2.hpp>
#include <ecn_baxter/game/game_properties.hpp>
#include <ecn_baxter/game/master/gui_management.hpp>
#include <ecn_baxter/game/properties_loader.hpp>
#include <ecn_baxter/game/ros1/game_master_1.hpp>
#include <ecn_baxter/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <ros/master.h>
#include <thread>

namespace ecn_baxter::game {
using namespace std::chrono_literals;

/// @brief Main class for game management and logic
class Game : GamePropertiesLoader, UIManager {
public:
  /**========================================================================
   **                            Cycle Utils
   *========================================================================**/
  inline static bool Init(int argc, char **argv) {
    return instance()->init(argc, argv);
  }
  /// @brief Run one cycle of the ROS nodes
  inline static void spinOnce() {
    instance()->ex->spin_once(10ms);
    ros::spinOnce();
  }

  /// @brief Send a stop signal to all ROS-related thread
  inline static void stop() {
    stop_cmd = 1;
    if (ros_thread.joinable())
      ros_thread.join();
    ros::shutdown();
    rclcpp::shutdown();
    clean();
  }

  /// @brief Cleaning procedure for erasing all left pointers that could be left
  inline static void clean() {
    game_props.reset();
    _instance.reset();
  }

  /**========================================================================
  **                            UI Management
  *========================================================================**/
  /// @brief Show the main GUI, make events binding, etc
  inline static void showUI() { instance()->show_ui(); };

protected:
  /**========================================================================
   *?                            ROS Node
   *========================================================================**/
  sptr<ros1::GameMaster_1> ros1_node;
  sptr<rclcpp::executors::SingleThreadedExecutor> ex;
  sptr<GameMaster_2> ros2_node;

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
  inline static sptr<Game> instance() {
    if (_instance == nullptr)
      _instance = std::make_shared<Game>();
    return _instance;
  }

private:
  /**========================================================================
   *!                           SINGLETON PATTERN
   *========================================================================**/
  Game();
  static sptr<Game> _instance;
  ~Game() {
    ros1_node.reset();
    ros2_node.reset();
    ex.reset();
  }
};
} // namespace ecn_baxter::game

#endif // GAME_HPP

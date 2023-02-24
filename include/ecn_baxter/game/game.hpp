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

#include <chrono>
#include <cstdlib>
#include <ecn_baxter/game/game_master_1.hpp>
#include <ecn_baxter/game/game_master_2.hpp>
#include <ecn_baxter/game/game_properties.hpp>
#include <ecn_baxter/game/properties_loader.hpp>
#include <ecn_baxter/ui/file_loader_wrapper.hpp>
#include <ecn_baxter/ui/main_wrapper.hpp>
#include <ecn_baxter/utils.hpp>
#include <rapidjson/document.h>
#include <rclcpp/rclcpp.hpp>
#include <ros/master.h>
#include <thread>

namespace ecn_baxter::game {
using namespace std::chrono_literals;
using rapidjson::Document;

/// @brief Main class for game management and logic
class Game : GamePropertiesLoader {
public:
  static bool init(int argc, char **argv);

  /**========================================================================
   *?                             ROS Node
   *========================================================================**/
  inline static GameMaster_1 *ros1() { return ros1_node.get(); }
  inline static GameMaster_2 *ros2() { return ros2_node.get(); }
  inline static std::thread *main_thread() { return &ros_thread; }

  /**========================================================================
   **                            Cycle Utils
   *========================================================================**/
  /// @brief Run one cycle of the ROS nodes
  inline static void spinOnce() {
    ex->spin_once(10ms);
    ros1_node->spin();
    ros::spinOnce();
  }

  /// @brief Send a stop signal to all ROS-related thread
  inline static void stop() { stop_cmd = 1; }

  /// @brief Return whether the ROS tasks are stopping or not
  /// @return whether or not it is stopping
  inline static bool isStopping() { return stop_cmd == 1; }

  /// @brief Cleaning procedure for erasing all left pointers that could be left
  inline static void clean() {
    ros2_node.reset();
    ex.reset();
    game_props.reset();
    ros1_node.reset();
    main_window.reset();
    bindings.reset();
  }

  /**========================================================================
  **                            UI Management
  *========================================================================**/
  /// @brief Show the main GUI, make events binding, etc
  static void show_gui();

  /// @brief Set the event callbacks for the UIs
  inline static void bind_gui() { _bind_gui(); }

protected:
  /**========================================================================
   *?                            ROS Node
   *========================================================================**/
  static sptr<GameMaster_1> ros1_node;
  static sptr<rclcpp::executors::SingleThreadedExecutor> ex;
  static sptr<GameMaster_2> ros2_node;

  /**========================================================================
   *?                           Game Management
   *========================================================================**/
  static void load_game_propeties(const std::string &);

  /**========================================================================
   **                            UI Management
   *========================================================================**/
  static sptr<gui::MainUI> main_window;
  static sptr<std::function<void()>> bindings;
  static void _bind_gui();

  /**========================================================================
   **                            Game Thread
   *========================================================================**/
  static std::thread ros_thread;
  static void ros_loop();
  static sig_atomic_t stop_cmd;
};
} // namespace ecn_baxter::game

#endif // GAME_HPP

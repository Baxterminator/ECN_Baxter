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

#include <ecn_baxter/game/game_master_1.hpp>
#include <ecn_baxter/game/game_master_2.hpp>
#include <ros/node_handle.h>
#include <ros/master.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <cstdlib>
#include <chrono>

namespace ecn_baxter::game
{
    using namespace std::chrono_literals;

    /// @brief Main class for game management and logic
    class Game
    {
    public:
        static bool init(int argc, char **argv);

        /**========================================================================
         *?                             ROS Node
         *========================================================================**/
        inline static ros::NodeHandle *ros1() { return ros1_node.get(); }
        inline static rclcpp::Node *ros2() { return ros2_node.get(); }
        inline static std::thread *main_thread() { return &ros_thread; }

        /**========================================================================
         **                            Cycle Utils
         *========================================================================**/
        /// @brief Run one cycle of the ROS nodes
        inline static void spinOnce()
        {
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
        inline static void clean()
        {
            timer.reset();
            ros2_node.reset();
            ex.reset();
        }

        /**========================================================================
         **                            Game Actions
         *========================================================================**/

        /// @brief Set the event callbacks for the UIs
        inline static void bind_gui()
        {
            ros2_node->bind_gui();
        }

    protected:
        /**========================================================================
         *?                            ROS Node
         *========================================================================**/
        static std::unique_ptr<GameMaster_1> ros1_node;
        static rclcpp::executors::SingleThreadedExecutor::SharedPtr ex;
        static std::shared_ptr<GameMaster_2> ros2_node;
        static rclcpp::TimerBase::SharedPtr timer;

        /**========================================================================
         **                            Game Thread
         *==========ROS1 part of the GameMaster==============================================================**/
        static std::thread ros_thread;
        static void ros_loop();
        static sig_atomic_t stop_cmd;
    };
}

#endif // GAME_HPP

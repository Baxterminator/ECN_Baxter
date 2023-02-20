/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Main class for game management and logic
 *========================================================================**/
#include <ecn_baxter/game/game.hpp>

namespace ecn_baxter::game
{
    std::unique_ptr<GameMaster_1> Game::ros1_node;
    std::shared_ptr<GameMaster_2> Game::ros2_node;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr Game::ex;
    rclcpp::TimerBase::SharedPtr Game::timer;
    std::thread Game::ros_thread;
    sig_atomic_t Game::stop_cmd;

    /// @brief Initialize everything related to the ROS 1&2 nodes
    /// @return true if the initialization as successful
    bool Game::init(int argc, char **argv)
    {
        // ROS 2 initialization
        rclcpp::init(argc, argv);

        stop_cmd = 0;

        ros2_node = std::make_shared<GameMaster_2>(rclcpp::NodeOptions{});

        ros::init(argc, argv, "game_master_1");
        ex = rclcpp::executors::SingleThreadedExecutor::make_shared();
        ex->add_node(ros2_node);
        /*
                // Checking if another game master is launched
                vector<string> nodes; ros::master::getNodes(nodes);
                auto it = find(nodes.begin(), nodes.end(), "game_master_1");
                if (it != nodes.end()) {
                    RCLCPP_FATAL(ros2()->get_logger(), "Game master is already launched !");
                    return false;
                }*/

        // ROS 1 initialization
        ros1_node = std::make_unique<GameMaster_1>();

        ros_thread = std::thread(Game::ros_loop);
        return true;
    }

    /// @brief Main task for ros loop
    void Game::ros_loop()
    {
        ros::AsyncSpinner async(1);
        async.start();
        while (ros::ok() && rclcpp::ok() && !stop_cmd)
        {
            Game::spinOnce();
        }
    }
}
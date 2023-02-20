/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS2 part of the GameMaster
 *========================================================================**/
#include <ecn_baxter/game/game_master_2.hpp>
#include <iostream>

namespace ecn_baxter::game
{

    GameMaster_2::GameMaster_2(rclcpp::NodeOptions opts) : rclcpp::Node("game_master_2", opts),
                                                           SetupPointsClient(get_logger())
    {
    }

    /// @brief Bind the UIs events to ROS tasks
    void GameMaster_2::bind_gui()
    {
        UIWrapper main_gui;
        QObject::connect(main_gui.setup(), &QPushButton::clicked, [this]()
                         { launch_setup(); });
        QObject::connect(main_gui.load_game(), &QPushButton::clicked, [this]()
                         { FileLoaderWrapper::display(); });
    }
}
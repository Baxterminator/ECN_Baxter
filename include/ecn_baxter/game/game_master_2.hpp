/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS2 part of the GameMaster
 *========================================================================**/

#ifndef GAME_MASTER_2_HPP
#define GAME_MASTER_2_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ecn_baxter/ui/main_wrapper.hpp>
#include <ecn_baxter/ui/file_loader_wrapper.hpp>
#include <ecn_baxter/action/points_setup.hpp>
#include <ecn_baxter/setup/client_points.hpp>

namespace ecn_baxter::game
{

    using ecn_baxter::action::PointsSetup;
    using namespace ecn_baxter::gui;

    class GameMaster_2 : public rclcpp::Node,
                         ecn_baxter::setup::SetupPointsClient
    {
    public:
        explicit GameMaster_2(rclcpp::NodeOptions opts);
        /**========================================================================
         **                            GUI Management
         *========================================================================**/
        void bind_gui();
    };
}
#endif // GAME_MASTER_2_HPP
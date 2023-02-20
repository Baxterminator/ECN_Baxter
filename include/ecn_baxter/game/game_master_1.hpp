/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS1 part of the GameMaster
 *========================================================================**/

#ifndef GAME_MASTER_1_HPP
#define GAME_MASTER_1_HPP

#include <ros/node_handle.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>

namespace ecn_baxter::game
{

    using geometry_msgs::TransformStamped;
    using geometry_msgs::msg::Point;

    class GameMaster_1 : public ros::NodeHandle
    {
    public:
        GameMaster_1() : ros::NodeHandle() {}
        void addPoint(const std::string &, const Point &);

        void spin();

    protected:
        ros::Publisher tf_pub = advertise<TransformStamped>("/tf_manual", 1000);
        std::vector<TransformStamped> points_to_broadcast;
        void broadcast_points();
    };
};

#endif // GAME_MASTER_1_HPP
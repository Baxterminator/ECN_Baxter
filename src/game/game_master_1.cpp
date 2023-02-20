/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  ROS1 part of the GameMaster
 *========================================================================**/
#include <ecn_baxter/game/game_master_1.hpp>

namespace ecn_baxter::game
{
    /// @brief Function to execute each cycle
    void GameMaster_1::spin()
    {
        broadcast_points();
    }

    /// @brief Add a marker to broadcast on the TF tree (/tf_manual)
    /// @param name the name of the marker
    /// @param ptn the position (x,y,z) of the marker to publish
    void GameMaster_1::addPoint(const std::string &name, const Point &point)
    {
        TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "base";
        ts.child_frame_id = name;

        ts.transform.translation.x = point.x;
        ts.transform.translation.y = point.y;
        ts.transform.translation.z = point.z;

        points_to_broadcast.push_back(ts);
    }

    /// @brief Broadcast all saved points
    void GameMaster_1::broadcast_points()
    {
        for (auto point : points_to_broadcast)
        {
            tf_pub.publish(point);
        }
    }
}
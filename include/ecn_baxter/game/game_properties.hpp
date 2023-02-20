/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Properties definitions of the games
 *========================================================================**/

#ifndef GAME_PROPERTIES_HPP
#define GAME_PROPERTIES_HPP

#include <string>
#include <vector>

enum class Side
{
    RIGHT_ARM,
    LEFT_ARM
};

struct Point
{
    std::string name;
    Side arm_side;
    bool w_angles = false;
};

struct SetupRequirements
{
    std::vector<Point> needed_points;
};

struct GameProperties
{
    std::string filename;
    SetupRequirements setup;
};

#endif // GAME_PROPERTIES_HPP
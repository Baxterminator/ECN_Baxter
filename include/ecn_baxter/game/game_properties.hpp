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

struct SetupRequirements
{
};

struct GameProperties
{
    std::string filename;
    SetupRequirements setup;
};

#endif
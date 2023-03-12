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

#include <ecn_baxter/game/data/arm_side.hpp>
#include <ostream>
#include <string>
#include <vector>

namespace ecn_baxter::game::data {
/**========================================================================
 **                       GAME PROPERTIES STRUCTS
 *========================================================================**/
struct GPoint {
  constexpr static auto NAME_TAG{"name"};
  std::string name;
  constexpr static auto ARM_TAG{"side"};
  ArmSide arm_side;
  constexpr static auto ANGLES_TAG{"angles"};
  bool w_angles = false;

  // Printing
  friend std::ostream &operator<<(std::ostream &, GPoint const &);
};

struct SetupRequirements {
  constexpr static auto NEEDED_PTS_TAG{"nd_pts"};
  std::vector<GPoint> needed_points = std::vector<GPoint>(0);
  inline bool is_skippable() { return needed_points.empty(); }

  // Printing
  friend std::ostream &operator<<(std::ostream &, SetupRequirements const &);
};

struct GameProperties {
  std::string filename;
  constexpr static auto NAME_TAG{"name"};
  std::string game_name;
  constexpr static auto SETUP_TAG{"setup"};
  SetupRequirements setup;
  inline bool is_skippable() { return setup.is_skippable(); }

  // Printing
  friend std::ostream &operator<<(std::ostream &, GameProperties const &);
};

} // namespace ecn_baxter::game::data

#endif // GAME_PROPERTIES_HPP
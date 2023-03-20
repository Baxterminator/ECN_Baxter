/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Game Properties definition
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_GAME_PROPERTIES_HPP
#define ECN_BAXTER_GAME_PROPERTIES_HPP

#include "ecn_baxter/game/data/arm_side.hpp"
#include <ostream>
#include <string>
#include <tf2_msgs/TFMessage.h>
#include <vector>

namespace ecn_baxter::game::data {

using tf2_msgs::TFMessage;

struct GPoint {
  constexpr static auto NAME_TAG{"name"};
  std::string name;
  constexpr static auto ARM_TAG{"side"};
  ArmSide arm_side;
  constexpr static auto ANGLES_TAG{"angles"};
  bool with_angles = false;

  // Printing
  friend std::ostream &operator<<(std::ostream &, GPoint const &);
};

struct SetupRequirements {
  constexpr static auto NEEDED_PTS_TAG{"nd_pts"};
  std::vector<GPoint> needed_points = std::vector<GPoint>(0);
  TFMessage ptn_msgs;
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

#endif
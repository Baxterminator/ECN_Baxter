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

#include <ostream>
#include <string>
#include <vector>

namespace ecn_baxter::game {
enum class Side { RIGHT_ARM, LEFT_ARM };

inline std::string side2str(Side s) {
  if (s == Side::RIGHT_ARM)
    return "right_arm";
  if (s == Side::LEFT_ARM)
    return "left_arm";
  return "";
}

inline bool side2bool(Side s) {
  switch (s) {
  case Side::LEFT_ARM:
    return true;
  case Side::RIGHT_ARM:
    return false;
  }
}

inline Side bool2side(bool b) { return (b) ? Side::LEFT_ARM : Side::RIGHT_ARM; }

struct GPoint {
  std::string name;
  Side arm_side;
  bool w_angles = false;

  friend std::ostream &operator<<(std::ostream &Str, GPoint const &s) {
    Str << s.name << "(" << side2str(s.arm_side) << ")"
        << "{ angles: " << s.w_angles << "}" << std::endl;
    return Str;
  }
};

struct SetupRequirements {
  std::vector<GPoint> needed_points = std::vector<GPoint>(0);

  friend std::ostream &operator<<(std::ostream &Str,
                                  SetupRequirements const &s) {
    for (auto p : s.needed_points) {
      Str << "\t- " << p;
    }
    return Str;
  }

  inline bool is_skippable() { return needed_points.empty(); }
};

struct GameProperties {
  std::string filename;
  std::string game_name;
  SetupRequirements setup;

  friend std::ostream &operator<<(std::ostream &Str, GameProperties const &v) {
    Str << "[" << v.game_name << "]" << std::endl;
    Str << "\t(" << v.filename << ")" << std::endl;
    Str << v.setup;
    return Str;
  }
  inline bool is_skippable() { return setup.is_skippable(); }
};
} // namespace ecn_baxter::game

#endif // GAME_PROPERTIES_HPP
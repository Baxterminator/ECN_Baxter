/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Properties definitions of the games
 *========================================================================**/
#include <ecn_baxter/game/data/game_properties.hpp>

namespace ecn_baxter::game::data {

/**========================================================================
 **                       GAME PROPERTIES PRINTING
 *========================================================================**/
std::ostream &operator<<(std::ostream &Str, GPoint const &s) {
  Str << s.name << "(" << side2str(s.arm_side) << ")"
      << "{ angles: " << s.w_angles << "}" << std::endl;
  return Str;
}

std::ostream &operator<<(std::ostream &Str, SetupRequirements const &s) {
  for (auto p : s.needed_points) {
    Str << "\t- " << p;
  }
  return Str;
}

std::ostream &operator<<(std::ostream &Str, GameProperties const &v) {
  Str << "[" << v.game_name << "]" << std::endl;
  Str << "\t(" << v.filename << ")" << std::endl;
  Str << v.setup;
  return Str;
}

} // namespace ecn_baxter::game::data
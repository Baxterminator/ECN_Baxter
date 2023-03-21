/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Arm Side enum definition
 * @version        :  rev rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_ARM_SIDE
#define ECN_BAXTER_ARM_SIDE

#include <string>

namespace ecn_baxter::game::data {

enum class ArmSide { NONE, RIGHT_ARM, LEFT_ARM, BOTH };

inline std::string side2str(ArmSide s) {
  switch (s) {
  case ArmSide::RIGHT_ARM:
    return "RIGHT";
  case ArmSide::LEFT_ARM:
    return "LEFT";
  case ArmSide::BOTH:
    return "RIGHT | LEFT";
  default:
    return "";
  }
}

inline bool side2bool(ArmSide s) {
  switch (s) {
  case ArmSide::LEFT_ARM:
    return true;
  case ArmSide::RIGHT_ARM:
    return false;
  default:
    return false;
  }
}

inline ArmSide bool2side(bool b) {
  return (b) ? ArmSide::LEFT_ARM : ArmSide::RIGHT_ARM;
}

} // namespace ecn_baxter::game::data

#endif
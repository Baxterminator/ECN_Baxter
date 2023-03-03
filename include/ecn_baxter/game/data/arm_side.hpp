/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Arm Side enum definition
 *========================================================================**/

#ifndef ARM_SIDE
#define ARM_SIDE

#include <string>

namespace ecn_baxter::game::data {
enum class ArmSide { NONE, RIGHT_ARM, LEFT_ARM };

inline std::string side2str(ArmSide s) {
  switch (s) {
  case ArmSide::NONE:
    return "";
  case ArmSide::RIGHT_ARM:
    return "right_arm";
  case ArmSide::LEFT_ARM:
    return "left_arm";
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
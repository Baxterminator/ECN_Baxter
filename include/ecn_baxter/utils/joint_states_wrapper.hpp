/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  28/03/2023
 * @description    :  Wrapper classes for Baxter joints states
 * @version        :  rev 23w12.4
 * ════════════════════════════════════════════════════════════════════════**/
#include <cstdlib>
#include <math.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

namespace ecn_baxter::utils {

using JS = sensor_msgs::msg::JointState;

/**================================================================================================
 *?                                         Joint Making
 *================================================================================================**/

const std::vector<std::string> joints_suffixes{"_s0", "_s1", "_e0", "_e1",
                                               "_w0", "_w1", "_w2"};

inline void make_names(const std::string side,
                       std::vector<std::string> &container) {
  container.resize(0);
  for (const auto &s : joints_suffixes)
    container.push_back(side + s);
}

/**================================================================================================
 *?                                         Joint Structures
 *================================================================================================**/

/// @brief Represent a joint with its position, velocity and the applied effort
struct Joint {
  std::string name;
  double pos = .0, vel = .0, eff = .0;
};

/// @brief Represent the joints of one of the Baxter's arms
struct ArmJoints {
  Joint s0, s1;
  Joint e0, e1;
  Joint w0, w1, w2;

  explicit ArmJoints(std::string side) {
    s0.name = side + joints_suffixes[0];
    s1.name = side + joints_suffixes[1];
    e0.name = side + joints_suffixes[2];
    e1.name = side + joints_suffixes[3];
    w0.name = side + joints_suffixes[4];
    w1.name = side + joints_suffixes[5];
    w2.name = side + joints_suffixes[6];
  }
};

/// @brief Represent the joints for the Baxter's head
struct HeadJoints {
  Joint nod{"head_nod"}, pan{"head_pan"};
};

/// @brief Represent all the joints of the Baxter arms
struct BaxterJoints {
  JS::SharedPtr _states;
  ArmJoints left{"left"}, right{"right"};
  HeadJoints head;

  void set_joint(int pos, Joint &joint) {
    joint.pos = _states->position[pos];
    joint.vel = _states->velocity[pos];
    joint.eff = _states->effort[pos];
  }

  BaxterJoints(){};
  BaxterJoints(const JS::SharedPtr states) { set(states); }

#define make_joint(j)                                                          \
  if (name == j.name) {                                                        \
    set_joint(i, j);                                                           \
    continue;                                                                  \
  }

  void set(const JS::SharedPtr states) {
    _states = states;
    for (unsigned long i = 0; i < states->name.size(); i++) {
      std::string name = states->name[i];
      make_joint(head.nod);
      make_joint(head.pan);
      make_joint(left.s0);
      make_joint(left.s1);
      make_joint(right.s1);
      make_joint(left.e0);
      make_joint(right.e0);
      make_joint(left.e1);
      make_joint(right.e1);
      make_joint(left.w0);
      make_joint(right.w0);
      make_joint(left.w1);
      make_joint(right.w1);
      make_joint(left.w2);
      make_joint(right.w2);
    }
  }

}; // namespace ecn_baxter::utils

} // namespace ecn_baxter::utils
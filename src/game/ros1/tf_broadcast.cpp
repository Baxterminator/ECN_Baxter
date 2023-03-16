/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  26/02/2023
 * @description    :  Sub-part of the ROS1 game master for publishing tf
 *                    custom transforms
 *========================================================================**/
#include "ros/publisher.h"
#include "ros/time.h"
#include "ros/wall_timer.h"
#include <ecn_baxter/game/ros1/tf_broadcast.hpp>

namespace ecn_baxter::game::ros1 {

void TFBroadcaster::tf_broadcast_init(std::weak_ptr<ros::NodeHandle> handle) {
  node_handle = handle;
}

/// @brief Publish all saved markers to the TF tree of the robot
void TFBroadcaster::broadcast_transforms() const {
  if (game_props.expired()) {
    ROS_WARN_NAMED("gm1_tf_pub", "Game Properties pointer expired !");
    return;
  }

  auto s_gprops = game_props.lock();

  // Header change
  for (auto &transform : s_gprops->setup.ptn_msgs.transforms) {
    transform.header.stamp = ros::Time::now();
  }

  _tf_publisher.publish(s_gprops->setup.ptn_msgs);
}
void TFBroadcaster::update_game_props(
    std::weak_ptr<data::GameProperties> gprops) {
  if (!node_handle.expired()) {
    auto sptr_node = node_handle.lock();
    _tf_broadcast_timer = sptr_node->createWallTimer(
        dur, [this]([[maybe_unused]] const ros::WallTimerEvent &event) {
          broadcast_transforms();
        });
    _tf_publisher = sptr_node->advertise<TFMessage>(TF_TOPIC, TF_QUEUE_SIZE);
  }

  game_props = gprops;
}

} // namespace ecn_baxter::game::ros1
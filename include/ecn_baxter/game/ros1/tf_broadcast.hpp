/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  26/02/2023
 * @description    :  Sub-part of the ROS1 game master for publishing tf
 *                    custom transforms
 *========================================================================**/
#ifndef TF_BROADCAST
#define TF_BROADCAST

#include "ecn_baxter/game/data/game_properties.hpp"
#include "ros/duration.h"
#include "ros/publisher.h"
#include "ros/wall_timer.h"
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <ros/node_handle.h>
#include <tf2_msgs/TFMessage.h>

namespace ecn_baxter::game::ros1 {
using geometry_msgs::msg::Point;
using tf2_msgs::TFMessage;

class TFBroadcaster {
private:
  //* Wall timer for publishing data
  const ros::WallDuration dur{0.1};
  ros::WallTimer _tf_broadcast_timer;
  void broadcast_transforms() const;

  //* Publisher
  static constexpr auto TF_TOPIC{"/tf_manual"};
  static constexpr auto TF_QUEUE_SIZE{10};
  ros::Publisher _tf_publisher;

  std::weak_ptr<data::GameProperties> game_props;
  std::weak_ptr<ros::NodeHandle> node_handle;

protected:
  explicit TFBroadcaster(){};
  void tf_broadcast_init(std::weak_ptr<ros::NodeHandle>);

public:
  void update_game_props(std::weak_ptr<data::GameProperties>);
};

} // namespace ecn_baxter::game::ros1

#endif
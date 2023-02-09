#ifndef NODE2_HPP
#define NODE2_HPP

#include <rclcpp/rclcpp.hpp>
#include <ecn_baxter/game/gui_wrapper.hpp>

namespace ECNBaxter {
/**
 * ROS 2 Node for the game master
*/
class GMROS2 : public rclcpp::Node {
public:
    explicit GMROS2(rclcpp::NodeOptions opts) : rclcpp::Node("game_master_2", opts) {}
    inline void bind_gui() {
        QObject::connect(UIWrapper::setup(), &QPushButton::clicked, [this]() {launch_setup();});
    }
protected:
    void launch_setup();
};

}
#endif
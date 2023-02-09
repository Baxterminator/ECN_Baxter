#ifndef NODE1_HPP
#define NODE1_HPP

#include <ros/node_handle.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>

namespace ECNBaxter {
    using geometry_msgs::msg::Point;
    using geometry_msgs::TransformStamped;

    class GMROS1 : public ros::NodeHandle {
    public:
        GMROS1() : ros::NodeHandle() {}
        void addPoint(const std::string &name, const Point &ptn);

        void spin();
    protected:
        ros::Publisher tf_pub = advertise<TransformStamped>("/tf_manual", 1000);
        std::vector<TransformStamped> point_to_broadcast;
        void broadcast_ptn();
    };
};

#endif // NODE1_HPP
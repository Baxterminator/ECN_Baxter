#ifndef BAXTER_LAB_ROS1_NODE_H
#define BAXTER_LAB_ROS1_NODE_H

#include "ros/node_handle.h"
#include "ros/master.h"
#include "rclcpp/rclcpp.hpp"

namespace ECNBaxter {

    using namespace std;

    class Node {
    public:
        static bool init(int argc, char** argv);
        inline static ros::NodeHandle* ros1() {return ros1_node.get();}
        inline static rclcpp::Node* ros2() {return ros2_node.get();}
        inline static void spinOnce() { ex->spin_once(10ms); }
        inline static void stop() {
            ros2_node.reset();
            ex.reset();
            rclcpp::shutdown();
        }
    protected:
        static unique_ptr<ros::NodeHandle> ros1_node;
        static rclcpp::executors::SingleThreadedExecutor::SharedPtr ex;
        static rclcpp::Node::SharedPtr ros2_node;
        static rclcpp::TimerBase::SharedPtr timer;
    };
}

#endif //BAXTER_LAB_ROS1_NODE_H

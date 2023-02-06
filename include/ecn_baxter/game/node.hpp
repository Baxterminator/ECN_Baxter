#ifndef BAXTER_LAB_ROS1_NODE_H
#define BAXTER_LAB_ROS1_NODE_H

#include <ros/node_handle.h>
#include <ros/master.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace ECNBaxter {

    using namespace std;

    class Node {
    public:
        static bool init(int argc, char** argv);
        inline static ros::NodeHandle* ros1() {return ros1_node.get();}
        inline static rclcpp::Node* ros2() {return ros2_node.get();}
        inline static thread* main_thread() {return &ros_thread;}
        inline static void spinOnce() { ex->spin_once(10ms); }
        inline static void shutdown() {
            ros2_node.reset();
            ex.reset();
            rclcpp::shutdown();
        }
        inline static void stop() {
            stop_cmd = 1;
        }

    protected:

        static thread ros_thread;
        static void ros_loop();
        static sig_atomic_t stop_cmd;

        static unique_ptr<ros::NodeHandle> ros1_node;
        static rclcpp::executors::SingleThreadedExecutor::SharedPtr ex;
        static rclcpp::Node::SharedPtr ros2_node;
        static rclcpp::TimerBase::SharedPtr timer;
    };
}

#endif //BAXTER_LAB_ROS1_NODE_H

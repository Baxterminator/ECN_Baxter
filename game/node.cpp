#include <ecn_baxter/game/node.hpp>
#include <iostream>
#include <vector>

namespace ECNBaxter {
    std::unique_ptr<ros::NodeHandle> Node::ros1_node{};
    rclcpp::Node::SharedPtr Node::ros2_node{};
    rclcpp::executors::SingleThreadedExecutor::SharedPtr Node::ex;
    rclcpp::TimerBase::SharedPtr Node::timer;
    thread Node::ros_thread;
    sig_atomic_t Node::stop_cmd;


    /**
     * Initialize everything related to the ROS 1&2 nodes
     * @return
     */
    bool Node::init(int argc, char** argv) {
        // ROS 2 initialization
        rclcpp::init(argc, argv);

        stop_cmd = 0;

        ros2_node = rclcpp::Node::make_shared("game_master_2", rclcpp::NodeOptions{});
        timer = ros2_node->create_wall_timer(10ms, [&](){
            RCLCPP_INFO(ros2_node->get_logger(), "Hello");
        });

        ros::init(argc, argv, "game_master_1");
        ex = rclcpp::executors::SingleThreadedExecutor::make_shared();
        ex->add_node(ros2_node);
/*
        // Checking if another game master is launched
        vector<string> nodes; ros::master::getNodes(nodes);
        auto it = find(nodes.begin(), nodes.end(), "game_master_1");
        if (it != nodes.end()) {
            RCLCPP_FATAL(ros2()->get_logger(), "Game master is already launched !");
            return false;
        }


        RCLCPP_INFO(ros2()->get_logger(), "Running Master on %s", std::getenv("ROS_MASTER_URI"));

        // ROS 1 initialization
        ros1_node = make_unique<ros::NodeHandle>();

*/
        ros_thread = thread(Node::ros_loop);
        return true;
    }

    /**
     * Main task for ros loop
     */
    void Node::ros_loop() {
        while (ros::ok() && rclcpp::ok() && !stop_cmd) {
            Node::spinOnce();
        }
        Node::shutdown();
    }
}
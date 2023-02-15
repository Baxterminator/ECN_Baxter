#include <ecn_baxter/game/node.hpp>

namespace ECNBaxter {
    std::unique_ptr<GMROS1> Node::ros1_node;
    std::shared_ptr<GMROS2> Node::ros2_node;
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

        ros2_node = std::make_shared<GMROS2>(rclcpp::NodeOptions{});

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
        }*/

        // ROS 1 initialization
        ros1_node = make_unique<GMROS1>();

        ros_thread = thread(Node::ros_loop);
        return true;
    }

    /**
     * Main task for ros loop
     */
    void Node::ros_loop() {
        ros::AsyncSpinner async(1);
        async.start();
        while (ros::ok() && rclcpp::ok() && !stop_cmd) {
            Node::spinOnce();
        }
    }

    void Node::clean() {
        timer.reset();
        ros2_node.reset();
        ex.reset();
    }
}
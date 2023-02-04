#include <ecn_baxter/game/node.hpp>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <chrono>

using namespace ECNBaxter;

volatile sig_atomic_t stop;

void sighandler(int s)
{
    stop = 1;
}

int main(int argc, char** argv) {

    signal(SIGINT, &sighandler);


    // Init the two nodes
    if (!Node::init(argc, argv))
        return -1;

    //ros::AsyncSpinner async(1);
    //async.start();

    // Main loop
    while (ros::ok() && rclcpp::ok() && !stop) {
        Node::spinOnce();
    }
    Node::stop();

    return 0;
}
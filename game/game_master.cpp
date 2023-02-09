#include <ecn_baxter/game/node.hpp>
#include <csignal>
#include "QtWebKit"
#include "ecn_baxter/game/gui_wrapper.hpp"
#include <memory>

using namespace ECNBaxter;

std::shared_ptr<QApplication> app = nullptr;

void sighandler(int s)
{
    app->quit();
}

int main(int argc, char** argv) {

    // Add callback on force quit to stop everything
    signal(SIGINT, &sighandler);

    // Init the two ros nodes
    if (!Node::init(argc, argv))
        return -1;

    // Launching GUI on main thread
    app = std::make_shared<QApplication>(argc, argv);
    UIWrapper::instance();
    UIWrapper::showW();
    Node::bind_gui();
    app->exec();
    app.reset();
    UIWrapper::clean();

    // Stop ROS 1&2 thread and wait for its end
    Node::stop();
    if (Node::main_thread()->joinable())
        Node::main_thread()->join();
    Node::clean();
    rclcpp::shutdown();

    return 0;
}
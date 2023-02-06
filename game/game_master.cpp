#include <ecn_baxter/game/node.hpp>
#include <csignal>
#include <ui_main.h>
#include "QtWebKit"

using namespace ECNBaxter;

shared_ptr<QApplication> app = nullptr;

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
    Ui_BaxterMaster gui;
    //QWebView::initialize();
    app = make_shared<QApplication>(argc, argv);
    QMainWindow widget;
    gui.setupUi(&widget);
    widget.show();
    app->exec();

    app.reset();

    RCLCPP_INFO(Node::ros2()->get_logger(), "Stop normally");

    // Stop ROS 1&2 thread and wait for its end
    Node::stop();
    if (Node::main_thread()->joinable())
        Node::main_thread()->join();

    return 0;
}
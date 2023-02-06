#include <ecn_baxter/game/node.hpp>
#include <csignal>
#include <ui_main.h>
#include "QtWebKit"

using namespace ECNBaxter;

volatile sig_atomic_t stop;

void sighandler(int s)
{
    stop = 1;
}

int main(int argc, char** argv) {

    signal(SIGINT, &sighandler);

    Ui_BaxterMaster gui;

    //QWebView::initialize();
    QApplication app(argc, argv);
    QMainWindow widget;


    gui.setupUi(&widget);
    widget.show();
    app.exec();

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
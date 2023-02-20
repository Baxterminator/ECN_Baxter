#include <ecn_baxter/game/game.hpp>
#include <csignal>
#include "QtWebKit"
#include "ecn_baxter/ui/main_wrapper.hpp"
#include <memory>

using namespace ecn_baxter::game;
using namespace ecn_baxter::gui;

std::shared_ptr<QApplication> app = nullptr;

void sighandler(int s)
{
    app->quit();
}

int main(int argc, char **argv)
{

    // Add callback on force quit to stop everything
    signal(SIGINT, &sighandler);

    // Init the two ros nodes
    if (!Game::init(argc, argv))
        return -1;

    // Launching GUI on main thread
    app = std::make_shared<QApplication>(argc, argv);

    // GUI Initialization
    UIWrapper main_gui;
    main_gui.instance();
    main_gui.show();
    Game::bind_gui();

    // Running GUI
    app->exec();

    // Cleaning procedure on stop
    app.reset();
    main_gui.clean();

    // Stop ROS 1&2 thread and wait for its end
    Game::stop();
    if (Game::main_thread()->joinable())
        Game::main_thread()->join();
    Game::clean();
    rclcpp::shutdown();

    return 0;
}
#include "QtWebKit"
#include "ecn_baxter/ui/file_loader_wrapper.hpp"
#include "ecn_baxter/ui/main_wrapper.hpp"
#include <csignal>
#include <ecn_baxter/game/game.hpp>
#include <memory>

using namespace ecn_baxter::game;
using namespace ecn_baxter::gui;

std::shared_ptr<QApplication> app = nullptr;

void sighandler(int s) { app->quit(); }

int main(int argc, char **argv) {

  // Add callback on force quit to stop everything
  signal(SIGINT, &sighandler);

  // Init the two ros nodes
  if (!Game::init(argc, argv))
    return -1;

  // Launching GUI on main thread
  app = std::make_shared<QApplication>(argc, argv);
  Game::show_gui();
  app->exec();
  app.reset();

  // Stop ROS 1&2 thread and wait for its end
  Game::stop();
  if (Game::main_thread()->joinable())
    Game::main_thread()->join();
  Game::clean();

  rclcpp::shutdown();

  return 0;
}
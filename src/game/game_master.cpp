#include "QtWebKit"
#include "ros/init.h"
#include <csignal>
#include <ecn_baxter/game/game.hpp>
#include <memory>

using namespace ecn_baxter::game;

std::shared_ptr<QApplication> app = nullptr;

void shutdown_process() {
  app->quit();
  app.reset();
  Game::stop();
}

void sighandler(int s) { shutdown_process(); }

int main(int argc, char **argv) {

  // Add callback on force quit to stop everything
  signal(SIGINT, &sighandler);

  // Init the two ros nodes
  if (!Game::Init(argc, argv))
    return -1;

  // Launching GUI on main thread
  app = std::make_shared<QApplication>(argc, argv);
  Game::showUI();
  app->exec();

  shutdown_process();
  return 0;
}
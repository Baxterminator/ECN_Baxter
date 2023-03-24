#include "QtWebKit"
#include "ecn_baxter/game/data/local_games.hpp"
#include "ecn_baxter/game/master/game.hpp"
#include "ecn_baxter/game/utils/logger.hpp"
#include "ros/init.h"
#include <csignal>
#include <iostream>
#include <memory>

using namespace ecn_baxter::game;

std::unique_ptr<Game> game_master = nullptr;

void shutdown_process() {
  std::cout << "Shutdown !" << std::endl;
  game_master->stop();
  game_master.reset();
}

void sighandler([[maybe_unused]] int s) { shutdown_process(); }

int main(int argc, char **argv) {

  // Add callback on force quit to stop everything
  signal(SIGINT, &sighandler);

  game_master = std::make_unique<Game>(argc, argv);

  ecn_baxter::utils::Logger::initialize(game_master->ros2());

  BAXTER_INFO("Test using baxter_info %f", 0.5);

  if (game_master->is_initialized()) {
    game_master->show_ui();
    game_master->exec();
  }

  shutdown_process();
  return 0;
}
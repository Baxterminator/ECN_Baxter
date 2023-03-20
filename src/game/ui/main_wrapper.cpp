/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  UI Wrapper for the main window
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/data/arm_side.hpp"
#include "ui_main.h"
#include <ecn_baxter/game/ui/main_wrapper.hpp>
#include <qcoreevent.h>
#include <qevent.h>
#include <qtablewidget.h>
#include <sstream>

namespace ecn_baxter::gui {

/**════════════════════════════════════════════════════════════════════════
 *!                           Initialization
 * ════════════════════════════════════════════════════════════════════════**/
MainUI::MainUI() : ecn::base::BaseGUI<Ui::BaxterMaster, QMainWindow>() {}

/**════════════════════════════════════════════════════════════════════════
 **                            Event Callbacks
 * ════════════════════════════════════════════════════════════════════════**/

void MainUI::setup_internal_callbacks() {
  gui->load_game->installEventFilter(this);
}

/// @brief Get all events and process internal events
bool MainUI::eventFilter(QObject *obj, QEvent *e) {
  if (obj == gui->load_game && e->type() == QEvent::MouseButtonRelease) {
    launch_game_loader();
    return true;
  }
  return false;
}

/**════════════════════════════════════════════════════════════════════════
 **                           External Callbacks
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Refresh the player list with the given
void MainUI::refresh_player_list(game::data::PlayerList &list) {
  // Check if GUI has been made
  if (gui == nullptr)
    return;

  // Show player list
  int connected = 0;
  for (auto &player : list) {
    if (player.new_discovered) {
      player.new_discovered = false;
      player.row_id = gui->users->rowCount();
      gui->users->insertRow(player.row_id);
      gui->users->setItem(player.row_id, 0,
                          new QTableWidgetItem(player.name.c_str()));
    }

    // Update what need to be updated
    if (player.connected)
      connected++;
    gui->users->setItem(player.row_id, 1,
                        new QTableWidgetItem((player.connected)
                                                 ? "Connected"
                                                 : "Not connected"));
    gui->users->setItem(
        player.row_id, 2,
        new QTableWidgetItem(game::data::side2str(player.side).c_str()));
  }

  // Update number of connected people
  std::stringstream ss;
  ss << connected;
  ss << "/";
  ss << list.size();
  gui->n_co->setText(ss.str().c_str());
}

/**════════════════════════════════════════════════════════════════════════
 *?                            Game Loader
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Setup the mgame selector menu
void MainUI::setup_game_loader() {
  game_loader = std::make_shared<FileLoaderWrapper>();
  game_loader->setup_ui();
}

/// @brief Launch the game selector menu
void MainUI::launch_game_loader() {
  if (game_loader == nullptr)
    setup_game_loader();
  game_loader->show();
}

} // namespace ecn_baxter::gui

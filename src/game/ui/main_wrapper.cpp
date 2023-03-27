/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  UI Wrapper for the main window
 * @version        :  rev 23w12.2
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/data/arm_side.hpp"
#include "ui_main.h"
#include <ecn_baxter/game/ui/main_wrapper.hpp>
#include <iostream>
#include <qaction.h>
#include <qcoreevent.h>
#include <qevent.h>
#include <qheaderview.h>
#include <qmenu.h>
#include <qnamespace.h>
#include <qtablewidget.h>
#include <sstream>

namespace ecn_baxter::gui {

/**════════════════════════════════════════════════════════════════════════
 *!                           Initialization
 * ════════════════════════════════════════════════════════════════════════**/
MainUI::MainUI() : ecn::base::BaseGUI<Ui::BaxterMaster, QMainWindow>() {}

/**════════════════════════════════════════════════════════════════════════
 *?                          UI custom settings
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Set the Users QTableWidget custom style parameters
void MainUI::set_users_view() {
  gui->users->viewport()->setContextMenuPolicy(
      Qt::ContextMenuPolicy::CustomContextMenu);

  gui->users->setColumnWidth(0, 160);
  gui->users->setColumnWidth(1, 130);
  gui->users->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
  gui->users->horizontalHeader()->setStretchLastSection(true);
  gui->users->horizontalHeader()->stretchLastSection();
  gui->users->setHorizontalScrollBarPolicy(
      Qt::ScrollBarPolicy::ScrollBarAlwaysOff);
}

/**════════════════════════════════════════════════════════════════════════
 **                            Event Callbacks
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Setup the internal UI callbacks between widgets
void MainUI::setup_internal_callbacks() {
  gui->load_game->installEventFilter(this);
  gui->users->viewport()->installEventFilter(this);

  set_custom_gui_properties();
}

/// @brief Get all events and process internal events
bool MainUI::eventFilter(QObject *obj, QEvent *e) {
#define event_is(t) e->type() == t
#define obj_is(t) obj == t
  if (obj_is(gui->load_game) && event_is(QEvent::MouseButtonRelease))
    return launch_game_loader();
  else if (obj_is(gui->users->viewport()) && event_is(QEvent::ContextMenu))
    return make_context_menu(e);
  return false;
}

/// @brief Make a context menu over an item
bool MainUI::make_context_menu(QEvent *e) {
  QContextMenuEvent *mouse_ev = static_cast<QContextMenuEvent *>(e);
  if (gui->users->itemAt(mouse_ev->pos()) == nullptr)
    return true;
  QTableWidgetItem *child_item =
      gui->users->item(gui->users->itemAt(mouse_ev->pos())->row(), 0);

  // Check if child exist
  if (child_item) {
    QMenu my_menu;

    QAction top, right, left, none;

    top.setText("Token to add");
    top.setSeparator(true);
    right.setText("Right arm");
    left.setText("Left arm");
    none.setText("None");

    my_menu.addAction(&top);
    my_menu.addAction(&right);
    my_menu.addAction(&left);
    my_menu.addAction(&none);
    my_menu.setFixedWidth(160);

    // If the user clicked on something, process the right action
    QAction *triggered = my_menu.exec(mouse_ev->globalPos());
    if (triggered != nullptr && !players.expired()) {

      // Getting the right player
      auto s_players = players.lock();
      auto player = s_players->players.begin();
      using game::data::ArmSide;
      while (player != s_players->players.end()) {

#define is_user() (player->name == child_item->text().toStdString())
#define R_sel() (triggered == &right)

        // Affecting arm to player (removing if he isn't the good one)
        if (triggered == &none && is_user())
          player->wanted_side = game::data::ArmSide::NONE;
        else {
          switch (player->wanted_side) {
          case ArmSide::NONE:
            player->wanted_side = (is_user()) ? ((R_sel()) ? ArmSide::RIGHT_ARM
                                                           : ArmSide::LEFT_ARM)
                                              : ArmSide::NONE;
            break;
          case ArmSide::RIGHT_ARM:
            player->wanted_side =
                (is_user()) ? ((R_sel()) ? ArmSide::RIGHT_ARM : ArmSide::BOTH)
                            : ((R_sel()) ? ArmSide::NONE : ArmSide::RIGHT_ARM);
            break;
          case ArmSide::LEFT_ARM:
            player->wanted_side =
                (is_user()) ? ((R_sel()) ? ArmSide::BOTH : ArmSide::LEFT_ARM)
                            : ((R_sel()) ? ArmSide::LEFT_ARM : ArmSide::NONE);
            break;
          case ArmSide::BOTH:
            player->wanted_side =
                (is_user())
                    ? ArmSide::BOTH
                    : ((R_sel()) ? ArmSide::LEFT_ARM : ArmSide::RIGHT_ARM);
            break;
          }
        }

        player++;
      }
    }
  }

  return true;
}

/**════════════════════════════════════════════════════════════════════════
 **                           External Callbacks
 * ════════════════════════════════════════════════════════════════════════**/

/// @brief Refresh the player list with the given
void MainUI::refresh_player_list() {
  // Check if GUI has been made and the player list has been propagated
  if (gui == nullptr || players.expired())
    return;

  auto list = players.lock();

  // Show player list
  int connected = 0;
  auto player = list->players.begin();
  while (player != list->players.end()) {
    if (player->new_discovered) {
      player->new_discovered = false;
      player->row_id = gui->users->rowCount();
      gui->users->insertRow(player->row_id);
      gui->users->setItem(player->row_id, 0,
                          new QTableWidgetItem(player->name.c_str()));
    }

    // Update what need to be updated
    if (player->connected)
      connected++;
    gui->users->setItem(player->row_id, 1,
                        new QTableWidgetItem((player->connected)
                                                 ? "Connected"
                                                 : "Not connected"));
    gui->users->setItem(player->row_id, 2,
                        new QTableWidgetItem(
                            game::data::side2str(player->wanted_side).c_str()));
    gui->users->item(player->row_id, 1)->setTextAlignment(Qt::AlignCenter);
    gui->users->item(player->row_id, 2)->setTextAlignment(Qt::AlignCenter);

    player++;
  }

  // Update number of connected people
  std::stringstream ss;
  ss << connected;
  ss << "/";
  ss << list->players.size();
  gui->n_co->setText(ss.str().c_str());

  // Updating SLAVE MODE status
  gui->slave_on->setText((list->is_slaving()) ? "ON" : "OFF");
  gui->slave->setText((list->is_slaving()) ? "Slave OFF" : "Slave ON");
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
bool MainUI::launch_game_loader() {
  if (game_loader == nullptr)
    setup_game_loader();
  game_loader->show();
  return true;
}

} // namespace ecn_baxter::gui

/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  GUI Wrapper for the main GUI
 *========================================================================**/
#include "ui_main.h"
#include <ecn_baxter/ui/main_wrapper.hpp>

namespace ecn_baxter::gui {

/**========================================================================
 *!                           INITIALIZATION
 *========================================================================**/
MainUI::MainUI(std::function<void(void)> binds)
    : BaseGUI<Ui::BaxterMaster, QMainWindow>(), bindings(binds) {}

/**========================================================================
 **                            EVENT Callbacks
 *========================================================================**/
void MainUI::setup_internal_callbacks() {
  gui->load_game->installEventFilter(this);
  //  connect(setup, &QPushButton::clicked, [this]() { launch_game_loader(); });
}

/// @brief Get all events and process internal events
bool MainUI::eventFilter(QObject *obj, QEvent *e) {
  if (obj == gui->load_game && e->type() == QEvent::MouseButtonRelease) {
    launch_game_loader();
    return true;
  }
  return false;
}

/**========================================================================
 **                            GAME Loader
 *========================================================================**/
/// @brief Setup the mgame selector menu
void MainUI::setup_game_loader() {
  game_loader = std::make_shared<FileLoaderWrapper>();
  game_loader->setup_ui();
  bindings();
}

/// @brief Launch the game selector menu
void MainUI::launch_game_loader() {
  if (game_loader == nullptr)
    setup_game_loader();
  game_loader->show();
}

} // namespace ecn_baxter::gui

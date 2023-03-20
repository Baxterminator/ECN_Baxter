/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  UI Wrapper for the main window
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_MAIN_WRAPPER_H
#define ECN_BAXTER_MAIN_WRAPPER_H

#include "ecn_baxter/base/base_gui.hpp"
#include "ecn_baxter/game/data/game_players.hpp"
#include "ecn_baxter/game/ui/file_loader_wrapper.hpp"
#include "ui_main.h"
#include <memory>
#include <qcoreevent.h>
#include <qmainwindow.h>
#include <qobject.h>

namespace ecn_baxter::gui {

/// @brief UI Wrapper for the main window
class MainUI : public ecn::base::BaseGUI<Ui::BaxterMaster, QMainWindow> {
protected:
  /**═════════════════════════════════════════════════════════════════════════
   *?                            Game Loader
   * ═════════════════════════════════════════════════════════════════════════**/

  std::shared_ptr<FileLoaderWrapper> game_loader = nullptr;
  void setup_game_loader();
  void launch_game_loader();

  /**═════════════════════════════════════════════════════════════════════════
   *?                            Event Callbacks
   * ═════════════════════════════════════════════════════════════════════════**/

  void setup_internal_callbacks() override;
  bool eventFilter(QObject *obj, QEvent *e) override;

public:
  explicit MainUI();
  FileLoaderWrapper *get_game_loader() { return game_loader.get(); }

  /**═════════════════════════════════════════════════════════════════════════
   *?                           External Callbacks
   * ═════════════════════════════════════════════════════════════════════════**/

  void refresh_player_list(game::data::PlayerList &);
};
} // namespace ecn_baxter::gui

#endif

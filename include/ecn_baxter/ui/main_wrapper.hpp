/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  GUI Wrapper for the main GUI
 *========================================================================**/

#ifndef MAIN_WRAPPER_H
#define MAIN_WRAPPER_H

#include "ui_main.h"
#include <ecn_baxter/ui/base_gui.hpp>
#include <ecn_baxter/ui/file_loader_wrapper.hpp>
#include <ecn_baxter/utils.hpp>
#include <functional>
#include <qcoreevent.h>
#include <qmainwindow.h>
#include <qobject.h>
#include <qpushbutton.h>

namespace ecn_baxter::gui {
class MainUI : public BaseGUI<Ui::BaxterMaster, QMainWindow> {
protected:
  /**========================================================================
   **                            GAME Loader
   *========================================================================**/
  sptr<FileLoaderWrapper> game_loader;
  std::function<void(void)> bindings;
  void setup_game_loader();
  void launch_game_loader();

  /**========================================================================
   **                            EVENT Callbacks
   *========================================================================**/
  void setup_internal_callbacks() override;
  bool eventFilter(QObject *obj, QEvent *e) override;

public:
  explicit MainUI(std::function<void(void)>);
  sptr<FileLoaderWrapper> get_game_loader() { return game_loader; }
  ~MainUI() { game_loader.reset(); }
};
} // namespace ecn_baxter::gui

#endif // MAIN_WRAPPER_H

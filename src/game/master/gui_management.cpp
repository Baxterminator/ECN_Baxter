/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Sub class for UI Logic
 *========================================================================**/
#include <ecn_baxter/game/master/gui_management.hpp>

namespace ecn_baxter::game {

UIManager::UIManager() {
  main_window =
      std::make_shared<gui::MainUI>(std::bind(&UIManager::_bind_ui, this));
}

/// @brief Show the main GUI
void UIManager::show_ui() {
  main_window->setup_ui();
  main_window->show();
}
} // namespace ecn_baxter::game
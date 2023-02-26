/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Sub class for UI Logic
 *========================================================================**/
#ifndef UI_MANAGEMENT
#define UI_MANAGEMENT

#include <ecn_baxter/ui/main_wrapper.hpp>
#include <ecn_baxter/utils.hpp>
#include <functional>

namespace ecn_baxter::game {
class UIManager {
protected:
  static sptr<gui::MainUI> main_window;
  static sptr<std::function<void()>> bindings;
  inline static void _bind_gui(){};

public:
  static void show_gui();
};
} // namespace ecn_baxter::game

#endif
/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  Parent class for all UIs related objects
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BASE_GUI_HPP
#define ECN_BASE_GUI_HPP

#include <memory>

namespace ecn::base {

/// @brief Base class for GUI Making defining parent container widget and show
/// method
/// @tparam UI the UI class to show
/// @tparam Parent the parent widget class that the UI is based on
template <class UI, class Parent> class BaseGUI : public Parent {
protected:
  std::shared_ptr<UI> gui;
  bool made = false;

  /// @brief Describe the internal callbacks to setup
  virtual void setup_internal_callbacks() = 0;

public:
  BaseGUI() : Parent() {}

  /// @brief Setup the UI and set the internal callbacks
  void setup_ui() {
    if (gui == nullptr)
      gui = std::make_shared<UI>();
    gui->setupUi(this);
    setup_internal_callbacks();
    made = true;
  }

  /// @brief Return whether the UI instance has been made successfully
  bool is_made() { return made; }

  /// @brief Return a pointer to the UI instance but don't call the UI setuping
  /// (return value may be nullptr)
  inline UI *get_ui() const { return gui.get(); }
};
} // namespace ecn::base

#endif
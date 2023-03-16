#ifndef BASE_GUI_HPP
#define BASE_GUI_HPP

#include <iostream>
#include <memory>

namespace ecn_baxter::base {
/**
 *@brief Base class for GUI Making defining parent container widget and show
 *method
 *
 *@tparam UI the UI class to show
 *@tparam Parent the parent widget class that the UI is based on
 */
template <class UI, class Parent> class BaseGUI : public Parent {
protected:
  std::shared_ptr<UI> gui;
  virtual void setup_internal_callbacks() = 0;

public:
  BaseGUI() : Parent() {}
  void setup_ui() {
    if (gui == nullptr)
      gui = std::make_shared<UI>();
    gui->setupUi(this);
    setup_internal_callbacks();
  }
  ~BaseGUI() { gui.reset(); }

  inline std::shared_ptr<UI> get_ui() const { return gui; }
};
} // namespace ecn_baxter::base

#endif
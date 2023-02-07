#include "ecn_baxter/game/gui_wrapper.hpp"


namespace ECNBaxter {
    std::shared_ptr<Ui_BaxterMaster> UIWrapper::gui_instance;
    std::shared_ptr<QMainWindow> UIWrapper::win;
    bool UIWrapper::initialized;

    std::shared_ptr<Ui_BaxterMaster> UIWrapper::instance() {
        if (gui_instance == nullptr) {
            gui_instance = std::make_shared<Ui_BaxterMaster>();
            initialized = false;
        }
        if (win == nullptr) {
            win = std::make_shared<QMainWindow>();
        }
        if (!initialized) {
            gui_instance->setupUi(win.get());
            initialized = true;
        }
        return gui_instance;
    }

    void UIWrapper::clean() {
        gui_instance.reset();
        win.reset();
    }
}
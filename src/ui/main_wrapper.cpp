#include "ecn_baxter/ui/main_wrapper.hpp"
#include "ui_main.h"

namespace ecn_baxter::gui
{
    sptr<Ui_BaxterMaster> UIWrapper::gui_instance;
    bool UIWrapper::initialized;
    sptr<QMainWindow> UIWrapper::win;

    QPushButton *UIWrapper::slave() { return instance()->slave; }
    QPushButton *UIWrapper::setup() { return instance()->setup; }
    QPushButton *UIWrapper::load_game() { return instance()->load_game; }

    sptr<Ui_BaxterMaster> UIWrapper::instance()
    {
        if (gui_instance == nullptr)
        {
            gui_instance = std::make_shared<Ui_BaxterMaster>();
            initialized = false;
        }
        if (win == nullptr)
        {
            win = std::make_shared<QMainWindow>();
        }
        if (!initialized)
        {
            gui_instance->setupUi(win.get());
            initialized = true;
        }
        return gui_instance;
    }

    void UIWrapper::clean()
    {
        gui_instance.reset();
        win.reset();
    }

} // namespace ecn_baxter

#include <ecn_baxter/ui/file_loader_wrapper.hpp>
#include "ui_game_loader.h"

namespace ECNBaxter{
    sptr<Ui_game_loader> FileLoaderWrapper::gui_instance;
    bool FileLoaderWrapper::initialized;
    sptr<QDialog> FileLoaderWrapper::win;

    sptr<Ui_game_loader> FileLoaderWrapper::instance() {
        if (gui_instance == nullptr) {
            gui_instance = std::make_shared<Ui_game_loader>();
            initialized = false;
        }
        if (win == nullptr) {
            win = std::make_shared<QDialog>();
        }
        if (!initialized) {
            gui_instance->setupUi(win.get());
            initialized = true;
        }
        return gui_instance;
    }

    void FileLoaderWrapper::clean() {
        gui_instance.reset();
        win.reset();
    }
 
} // namespace ECNBaxter

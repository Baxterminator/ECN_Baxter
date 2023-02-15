#ifndef FILE_LOADER_WRAPPER_H
#define FILE_LOADER_WRAPPER_H

#include "ui_game_loader.h"
#include <qt5/QtWidgets/QDialog>
#include <memory>

template<typename T>
using sptr = std::shared_ptr<T>;

namespace ECNBaxter {
    class FileLoaderWrapper {
    protected:
        static sptr<Ui_game_loader> gui_instance;
        static bool initialized;
        static sptr<QDialog> win;
    public:
        inline void show() {
            instance();
            win->show();
        }
        sptr<Ui_game_loader> instance();
        void clean();
    };
}
#endif //FILE_LOADER_WRAPPER_H
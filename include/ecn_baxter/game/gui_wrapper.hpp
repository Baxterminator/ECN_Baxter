#ifndef PROJ1_GUI_WRAPPER_H
#define PROJ1_GUI_WRAPPER_H

#include <iostream>
#include <memory>

namespace ECNBaxter {
class UIWrapper {
private:
    static std::shared_ptr<Ui_BaxterMaster> gui_instance;
    static bool initialized;
    static std::shared_ptr<QMainWindow> win;
public:
    inline static void showW() {win->show();}
    inline static QPushButton* slave() {return instance()->slave;}
    static std::shared_ptr<Ui_BaxterMaster> instance();
    static void clean();
};
}

#endif //PROJ1_GUI_WRAPPER_H

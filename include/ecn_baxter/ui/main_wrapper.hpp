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
#include <iostream>
#include <memory>

template <typename T>
using sptr = std::shared_ptr<T>;

namespace ecn_baxter::gui
{
    class UIWrapper
    {
    protected:
        static sptr<Ui_BaxterMaster> gui_instance;
        static bool initialized;
        static sptr<QMainWindow> win;

    public:
        inline void show()
        {
            instance();
            win->show();
        }
        sptr<Ui_BaxterMaster> instance();
        void clean();
        QPushButton *slave();
        QPushButton *setup();
        QPushButton *load_game();
    };
}

#endif // MAIN_WRAPPER_H

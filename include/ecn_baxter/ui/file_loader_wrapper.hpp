#ifndef FILE_LOADER_WRAPPER_H
#define FILE_LOADER_WRAPPER_H

#include "ui_game_loader.h"
#include <qt5/QtWidgets/QDialog>
#include <qt5/QtCore/QStringListModel>
#include <qt5/QtCore/QStringList>
#include <memory>
#include <map>
#include <iostream>

template<typename T>
using sptr = std::shared_ptr<T>;

namespace ECNBaxter {
    class FileLoaderWrapper : QDialog {
    protected:
        // #################################################
        //                   Class Working
        // #################################################
        static sptr<Ui_game_loader> gui_instance;
        static sptr<FileLoaderWrapper> obj_instance;
        static void setup_members();

        // #################################################
        //                  , COMPONENTS
        // #################################################
        void setup_gui();
            void setup_list();
            void setup_internal_action();
        bool eventFilter(QObject *obj, QEvent *event) override;

        QStringListModel *model;
        
    public:
        // #################################################
        //                      INIT
        // #################################################
        explicit FileLoaderWrapper();
        static void display();
        static sptr<Ui_game_loader> instance();
        static sptr<FileLoaderWrapper> get_instance();
        static void clean();

        virtual ~FileLoaderWrapper() {}

        // #################################################
        //                  , COMPONENTS
        // #################################################
        static QListView* list_game();
        static std::map<std::string, QRadioButton*> choices;
    };
}
#endif //FILE_LOADER_WRAPPER_H
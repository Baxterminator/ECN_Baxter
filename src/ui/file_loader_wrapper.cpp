/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  GUI Wrapper for the Game Choosing QDialog
 *========================================================================**/
#include <ecn_baxter/ui/file_loader_wrapper.hpp>
#include "ui_game_loader.h"

#include <QMouseEvent>
#include <iostream>

namespace ecn_baxter::gui
{
    sptr<Ui_game_loader> FileLoaderWrapper::gui_instance;
    sptr<FileLoaderWrapper> FileLoaderWrapper::obj_instance;
    std::map<std::string, QRadioButton *> FileLoaderWrapper::choices;

    /*  ################################################################

                                Windows Back part

        ################################################################*/
    FileLoaderWrapper::FileLoaderWrapper() : QDialog()
    {
    }
    sptr<FileLoaderWrapper> FileLoaderWrapper::get_instance()
    {
        if (obj_instance == nullptr)
        {
            obj_instance = std::make_shared<FileLoaderWrapper>();
        }
        return obj_instance;
    }
    sptr<Ui_game_loader> FileLoaderWrapper::instance()
    {
        if (gui_instance == nullptr)
        {
            gui_instance = std::make_shared<Ui_game_loader>();
            gui_instance->setupUi(get_instance().get());
            setup_members();
        }
        return gui_instance;
    }

    /*  ################################################################

                                Components

        ################################################################*/
    void FileLoaderWrapper::setup_members()
    {
        choices = {
            {"install", instance()->install_game},
            {"custom", instance()->custom_game}};
    }
    QListView *FileLoaderWrapper::list_game() { return instance()->list_files; }

    /*  ################################################################

                                Setuping GUI

        ################################################################*/
    void FileLoaderWrapper::clean()
    {
        gui_instance.reset();
        obj_instance.reset();
    }

    void FileLoaderWrapper::display()
    {
        instance();
        get_instance()->setup_gui();
        get_instance()->show();
        // get_instance()->launched = true;
    }

    void FileLoaderWrapper::setup_gui()
    {
        setup_list();
        setup_internal_action();
    }
    void FileLoaderWrapper::setup_internal_action()
    {
        list_game()->viewport()->installEventFilter(this);
    }
    void FileLoaderWrapper::setup_list()
    {
        model = new QStringListModel(this);
        QStringList l;
        l << "Test 1"
          << "Test 2";
        model->setStringList(l);

        list_game()->setModel(model);
        list_game()->setEditTriggers(QAbstractItemView::EditTrigger::NoEditTriggers);
    }

    bool FileLoaderWrapper::eventFilter(QObject *obj, QEvent *event)
    {

        if (obj == list_game()->viewport() && event->type() == QEvent::MouseButtonDblClick)
        {
            QMouseEvent *ev = static_cast<QMouseEvent *>(event);
            auto idx = list_game()->indexAt(ev->pos());

            if (!idx.isValid())
            {
                std::cout << "Double clicked outside" << std::endl;
                return false;
            }

            std::cout << "Double clicked at : " << idx.data().toString().toStdString() << std::endl;
        }
        return false;
    };

}

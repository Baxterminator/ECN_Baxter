/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  GUI Wrapper for the Game Choosing QDialog
 *========================================================================**/
#include "ui_game_loader.h"
#include <ecn_baxter/ui/file_loader_wrapper.hpp>
#include <qfiledialog.h>

namespace ecn_baxter::gui {

FileLoaderWrapper::FileLoaderWrapper() : BaseGUI<Ui::game_loader, QDialog>() {

  //  setup_file_browser();
}

/**========================================================================
 **                            Components
 *========================================================================**/

/**
 * @brief Return the path to the pre-installed selected description file
 *
 * @param ev the mouse event on click to set a new path
 * @return std::string the absolute path to the file
 */
std::string
FileLoaderWrapper::get_install_file_path(QMouseEvent *ev = nullptr) {
  // if event isn't a nullptr, update the install path to the new selected game
  if (ev != nullptr) {
    auto share = get_package_share_directory("ecn_baxter");
    auto idx = gui->list_files->indexAt(ev->pos());
    if (idx.isValid()) {
      install_path = share;
      install_path += "/games/";
      install_path += idx.data().toString().toStdString();
      install_path += ".bgame";
    }
  }

  return install_path;
}
/**
 * @brief Return the path to the custom selected description file
 *
 * @return std::string the absolute path to the file
 */
std::string FileLoaderWrapper::get_custom_file_path(bool set = false) {
  if (set) {
    QFileDialog file_browser(this);
    file_browser.setFileMode(QFileDialog::ExistingFile);
    file_browser.setNameFilter(tr("Baxter Game (*.bgame)"));

    if (file_browser.exec()) {
      auto list = file_browser.selectedFiles();
      if (list.size() > 0)
        custom_path = list.at(0).toStdString();
    }
  }

  return custom_path;
}
/**
 * @brief Return the path to the game_properties.game file to load
 *
 * @return std::string the absolute path to the file
 */
std::string FileLoaderWrapper::get_file_to_load() {
  if (gui->install_game->isChecked())
    return get_install_file_path();
  return get_custom_file_path();
}

/**========================================================================
 **                            Setuping GUI
 *========================================================================**/

void FileLoaderWrapper::setup_internal_callbacks() {
  setup_list();
  gui->list_files->viewport()->installEventFilter(this);
  gui->browse_file->installEventFilter(this);
}

/**
 * @brief Setup the file list with the pre-installed files
 */
void FileLoaderWrapper::setup_list() {
  model = new QStringListModel(this);
  QStringList l;
  l << "test1"
    << "Test 2";
  model->setStringList(l);

  gui->list_files->setModel(model);
  gui->list_files->setEditTriggers(
      QAbstractItemView::EditTrigger::NoEditTriggers);
}

/**========================================================================
 **                            Events Managment
 *========================================================================**/
/**
 * @brief Managing all the internal events of the GUI
 *
 * @param obj the object launching the event
 * @param event the launched event
 */
bool FileLoaderWrapper::eventFilter(QObject *obj, QEvent *event) {
  if (obj == gui->list_files->viewport() &&
      event->type() == QEvent::MouseButtonPress) {
    QMouseEvent *ev = static_cast<QMouseEvent *>(event);

    get_install_file_path(ev);
    return true;
  }
  if (obj == gui->browse_file && event->type() == QEvent::MouseButtonRelease) {
    gui->custom_file->setText(tr(get_custom_file_path(true).c_str()));
    gui->custom_game->setChecked(true);
    return true;
  }
  return false;
};

} // namespace ecn_baxter::gui

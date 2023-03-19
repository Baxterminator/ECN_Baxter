/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  GUI Wrapper for the Game Choosing QDialog
 *========================================================================**/

#ifndef FILE_LOADER_WRAPPER_H
#define FILE_LOADER_WRAPPER_H

#include "ui_game_loader.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ecn_baxter/base/base_gui.hpp>
#include <ecn_baxter/utils.hpp>
#include <map>
#include <qdialog.h>
#include <qdialogbuttonbox.h>
#include <qevent.h>
#include <qfiledialog.h>
#include <qpushbutton.h>
#include <qt5/QtCore/QStringList>
#include <qt5/QtCore/QStringListModel>
#include <qt5/QtWidgets/QDialog>

using namespace ament_index_cpp;

namespace ecn_baxter::gui {
class FileLoaderWrapper : public base::BaseGUI<Ui::game_loader, QDialog> {
protected:
  /**========================================================================
   **                            Components
   *========================================================================**/
  void setup_list();
  // void setup_file_browser();

  void setup_internal_callbacks() override;
  bool eventFilter(QObject *, QEvent *) override;

  QStringListModel *model;

  std::string install_path = "", custom_path = "";
  std::string get_install_file_path(QMouseEvent *);
  std::string get_custom_file_path(bool);

public:
  FileLoaderWrapper();
  std::string get_file_to_load();
};
} // namespace ecn_baxter::gui
#endif // FILE_LOADER_WRAPPER_H
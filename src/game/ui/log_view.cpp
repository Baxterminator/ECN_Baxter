/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  UI Logging view
 * @version        :  rev 23w12.4
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/ui/log_view.hpp"
#include <clocale>
#include <ctime>
#include <cwchar>
#include <qheaderview.h>
#include <qlist.h>
#include <qnamespace.h>
#include <qstandarditemmodel.h>
#include <qtableview.h>
#include <qtablewidget.h>
#include <rclcpp/logger.hpp>
#include <string>

namespace ecn_baxter::gui {

LogView::LogView(QTableWidget *view) : view_(view) {
  view_->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  set_header();
}

void LogView::set_header() {
  view_->setColumnCount(3);
  view_->setColumnWidth(0, 60);
  view_->setColumnWidth(1, 50);

  view_->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
}

void LogView::event_callback([[maybe_unused]] LogEvent *ev) {
  int row = view_->rowCount();
  view_->insertRow(row);

  view_->setItem(row, 0, new QTableWidgetItem(get_time().c_str()));
  view_->setItem(row, 1,
                 new QTableWidgetItem(get_level(ev->get_level()).c_str()));
  view_->setItem(row, 2, new QTableWidgetItem(ev->get_text().c_str()));
  view_->resizeRowToContents(row);

  while (view_->rowCount() > N_MAX_ROW) {
    view_->removeRow(0);
  }

  view_->scrollToBottom();
}

std::string LogView::get_time() {
  auto t = std::time(nullptr);
  char buff[30];
  std::strftime(buff, 30, "%OH:%OM:%OS", std::localtime(&t));
  return buff;
}

std::string LogView::get_level(rclcpp::Logger::Level lvl) {
  using Level = rclcpp::Logger::Level;

  switch (lvl) {
  case Level::Debug:
    return "DEBUG";
  case Level::Info:
    return "INFO";
  case Level::Warn:
    return "WARN";
  case Level::Error:
    return "ERROR";
  case Level::Fatal:
    return "FATAL";
  case Level::Unset:
    return "UNSET";
  }
}

} // namespace ecn_baxter::gui
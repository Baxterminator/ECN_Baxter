/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/02/2023
 * @description    :  UI Logging view
 * @version        :  rev 23w12.4
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_LOG_VIEW
#define ECN_BAXTER_LOG_VIEW

#include "ecn_baxter/game/events/log_event.hpp"
#include <qheaderview.h>
#include <qstandarditemmodel.h>
#include <qtablewidget.h>

namespace ecn_baxter::gui {

using game::events::LogEvent;

class LogView {
public:
  explicit LogView(QTableWidget *);

  void event_callback(LogEvent *);

private:
  static constexpr auto N_MAX_ROW{30};
  QTableWidget *view_;

  void set_header();

  std::string get_time();

  std::string get_level(rclcpp::Logger::Level lvl);
};

} // namespace ecn_baxter::gui

#endif
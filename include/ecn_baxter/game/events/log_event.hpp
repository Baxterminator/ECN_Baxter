/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  Event launched when something has to be logged
 * @version        :  rev 23w12.1
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_LOGGING
#define ECN_BAXTER_LOGGING

#include <qcoreevent.h>
#include <rclcpp/logger.hpp>

namespace ecn_baxter::game::events {

using Level = rclcpp::Logger::Level;

class LogEvent : public QEvent {
public:
  explicit LogEvent(Level level, const std::string &text)
      : QEvent(custom_type), level_(level), text_(text) {}

  static QEvent::Type type() { return custom_type; }

  Level get_level() { return level_; }
  const std::string &get_text() { return text_; }

private:
  const static QEvent::Type custom_type;
  Level level_;
  const std::string text_;
};

} // namespace ecn_baxter::game::events

#endif
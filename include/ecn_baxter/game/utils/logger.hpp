/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  Logging wrapper for the game master
 * @version        :  rev 23w12.4
 * ════════════════════════════════════════════════════════════════════════**/
#ifndef ECN_BAXTER_LOGGER
#define ECN_BAXTER_LOGGER

#include "ecn_baxter/game/events/event_target.hpp"
#include "ecn_baxter/game/events/log_event.hpp"
#include <qapplication.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace ecn_baxter::utils {

class Logger {
private:
  static rclcpp::Node *node_handle;

public:
  static void initialize(rclcpp::Node *);

  static void debug(const std::string &);
  static void info(const std::string &);
  static void warn(const std::string &);
  static void error(const std::string &);
  static void fatal(const std::string &);

  static std::string format_string(const char *fmt, ...) {
    va_list args1;
    va_start(args1, fmt);
    va_list args2;
    va_copy(args2, args1);
    std::vector<char> buf(1 + std::vsnprintf(nullptr, 0, fmt, args1));
    va_end(args1);
    std::vsnprintf(buf.data(), buf.size(), fmt, args2);
    va_end(args2);
    return buf.data();
  }
};

} // namespace ecn_baxter::utils

#define Log ecn_baxter::utils::Logger

#define BAXTER_DEBUG(...)                                                      \
  do {                                                                         \
    Log::debug(Log::format_string(__VA_ARGS__));                               \
  } while (0)
#define BAXTER_INFO(...)                                                       \
  do {                                                                         \
    Log::info(Log::format_string(__VA_ARGS__));                                \
  } while (0)
#define BAXTER_WARN(...)                                                       \
  do {                                                                         \
    Log::warn(Log::format_string(__VA_ARGS__));                                \
  } while (0)
#define BAXTER_ERROR(...)                                                      \
  do {                                                                         \
    Log::error(Log::format_string(__VA_ARGS__));                               \
  } while (0)
#define BAXTER_FATAL(...)                                                      \
  do {                                                                         \
    Log::fatal(Log::format_string(__VA_ARGS__));                               \
  } while (0)

#endif
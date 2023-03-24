/**════════════════════════════════════════════════════════════════════════
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  Logging wrapper for the game master
 * @version        :  rev 23w12.4
 * ════════════════════════════════════════════════════════════════════════**/
#include "ecn_baxter/game/utils/logger.hpp"

namespace ecn_baxter::utils {

rclcpp::Node *Logger::node_handle;

using game::events::EventTarget;
using game::events::LogEvent;
using Level = rclcpp::Logger::Level;

/// @brief Initialize the logger
void Logger::initialize(rclcpp::Node *node) { node_handle = node; }

void Logger::debug(const std::string &text) {
  if (node_handle == nullptr)
    return;
  RCLCPP_DEBUG(node_handle->get_logger(), "%s", text.c_str());
  QApplication::sendEvent(EventTarget::instance(),
                          new LogEvent(Level::Debug, text));
}

void Logger::info(const std::string &text) {
  if (node_handle == nullptr)
    return;
  RCLCPP_INFO(node_handle->get_logger(), "%s", text.c_str());
  QApplication::sendEvent(EventTarget::instance(),
                          new LogEvent(Level::Info, text));
}

void Logger::warn(const std::string &text) {
  if (node_handle == nullptr)
    return;
  RCLCPP_WARN(node_handle->get_logger(), "%s", text.c_str());
  QApplication::sendEvent(EventTarget::instance(),
                          new LogEvent(Level::Warn, text));
}

void Logger::error(const std::string &text) {
  if (node_handle == nullptr)
    return;
  RCLCPP_ERROR(node_handle->get_logger(), "%s", text.c_str());
  QApplication::sendEvent(EventTarget::instance(),
                          new LogEvent(Level::Error, text));
}

void Logger::fatal(const std::string &text) {
  if (node_handle == nullptr)
    return;
  RCLCPP_FATAL(node_handle->get_logger(), "%s", text.c_str());
  QApplication::sendEvent(EventTarget::instance(),
                          new LogEvent(Level::Fatal, text));
}

} // namespace ecn_baxter::utils
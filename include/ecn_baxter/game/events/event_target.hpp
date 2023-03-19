/**========================================================================
 * ?                                ABOUT
 * @author         :  Geoffrey Côte
 * @email          :  geoffrey.cote@centraliens-nantes.org
 * @repo           :  https://github.com/Baxterminator/ecn_baxter/
 * @createdOn      :  19/03/2023
 * @description    :  QObject Target for custom events that don't have a real
 *                    QObject specific target
 *========================================================================**/
#ifndef ECN_BAXTER_EVENT_TARGET
#define ECN_BAXTER_EVENT_TARGET

#include <iostream>
#include <memory>
#include <qwidget.h>

namespace ecn_baxter::game::events {

class EventTarget : public QWidget {
public:
  EventTarget() : QWidget() {}

  static EventTarget *instance() {
    static std::unique_ptr<EventTarget> _instance_obj =
        std::make_unique<EventTarget>();
    return _instance_obj.get();
  }
};

} // namespace ecn_baxter::game::events

#endif